#!/usr/bin/env python3
"""
drone_control/scripts/controllers/predictive_controller.py
Model predictive controller with optional neural network trajectory prediction.

NOTE ON THE NEURAL NETWORK PATH (read before enabling):
The network only produces meaningful predictions once real trained weights
are loaded from disk (see `~model_weights_path`). Building an untrained
Sequential model and calling .predict() on it (the original behaviour)
returns structured noise, not a prediction -- Keras initializes weights
randomly and nothing in this file ever calls .fit(). Until you have a
trained weights file, neural_network_enabled stays effectively off and the
controller uses `simple_target_prediction` (constant-velocity extrapolation)
instead, which is a legitimate, well-understood baseline on its own.
"""

import json
import os
import time
from enum import Enum

import numpy as np
import rospy


from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

from drone_control.lib import ControlLibrary
from drone_control.utils.ros_tracing import get_component_tracer, create_traced_publisher, traced_ros_callback, \
    span_context
from drone_control.utils.tracing import init_tracing, traced
from drone_control.msg  import TrackedTargets
from drone_control.utils import ErrorHandler
from drone_control.utils.correlation import get_or_create_mission_id
from drone_control.utils.logging_framework import get_logger,get_logger_with_ros_level
from drone_control.utils.metrics_collector import MetricsCollector, MetricTimer

# TensorFlow is optional at runtime. Importing it (or a wheel compiled for
# AVX on a CPU/VM without AVX) can hard-crash the process with "Illegal
# instruction" before any Python exception handling even runs. We isolate
# the import so a missing/incompatible TF install degrades this node to
# the non-neural fallback instead of taking the whole controller down.
TENSORFLOW_AVAILABLE = False
try:
    import tensorflow as tf
    TENSORFLOW_AVAILABLE = True
except Exception as _tf_import_error:  # ImportError, or a lower-level crash surfaced as SystemError/OSError on some platforms
    tf = None


class PredictiveControllerStatus(Enum):
    IDLE = "idle"
    PREDICTING = "predicting"
    OPTIMIZING = "optimizing"
    CONTROLLING = "controlling"
    EMERGENCY = "emergency"


class PredictiveController:
    """Model predictive controller with optional neural network trajectory prediction"""

    def __init__(self):
        rospy.init_node('predictive_controller', anonymous=False)

        # Initialize structured logger
        self.logger = get_logger_with_ros_level("predictive_controller")
        self.logger.info("node_initializing", extra={
            "version": "1.0.0",
            "simulation_mode": rospy.get_param('/use_simulation', True)
        })

        # Tracing setup, before anything else registers
        # publishers/subscribers/timers.
        init_tracing(component='predictive_controller', service_name='predictive-controller')
        self.tracer = get_component_tracer('predictive_controller')

        self.error_handler = ErrorHandler(node_name='predictive_controller')
        self.control_lib = ControlLibrary()

        # Configuration parameters
        self.horizon_length = rospy.get_param('~horizon_length', 10)
        self.time_step = rospy.get_param('~time_step', 0.1)
        self.control_frequency = rospy.get_param('~control_frequency', 50.0)
        self.max_velocity = rospy.get_param('~max_velocity', 5.0)
        self.max_acceleration = rospy.get_param('~max_acceleration', 2.0)
        self.max_jerk = rospy.get_param('~max_jerk', 1.0)
        self.safety_distance = rospy.get_param('~safety_distance', 1.0)
        self.optimization_weight = rospy.get_param('~optimization_weight', 1.0)

        # Neural network is requested via this param, but only actually
        # activates if TensorFlow imported successfully AND a trained
        # weights file was found and loaded. See initialize_neural_network().
        self.neural_network_requested = rospy.get_param('~neural_network_enabled', True)
        self.neural_network_enabled = False
        self.model_weights_path = rospy.get_param('~model_weights_path', '')

        self.simulation_mode = rospy.get_param('/use_simulation', True)

        # Initialize metrics collector (parameters above must be set first)
        self.metrics = MetricsCollector("predictive_controller", port=8006)
        self._init_metrics()

        # State variables
        self.current_state = None
        self.target_trajectory = None
        self.optimization_result = None
        self.neural_network_model = None
        self.prediction_horizon = []
        self.control_sequence = []
        self.status = PredictiveControllerStatus.IDLE
        self.last_update_time = 0.0
        self.emergency_stop = False
        self.prediction_errors = []
        self.control_performance = []
        self.trajectory_quality = 0.0

        # Subscribers
        self.odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odometry_callback)
        # NOTE: this used to import/subscribe with `TrackedTarget` (singular),
        # but the callback below reads msg.targets -- that's the *plural*
        # container message (see trajectory_planner.py, which gets this
        # right). Subscribing with the wrong message type makes ROS refuse
        # the connection outright ("topic types do not match"), so this
        # callback never fired at all and target-trajectory prediction never
        # ran.
        self.tracked_targets_sub = rospy.Subscriber('/tracked_targets', TrackedTargets, self.tracked_targets_callback)
        self.planner_status_sub = rospy.Subscriber('/planner_status', String, self.planner_status_callback)

        # Publishers
        self.control_pub = create_traced_publisher(
            '/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10, tracer_name='predictive_controller'
        )
        self.position_target_pub = create_traced_publisher(
            '/mavros/setpoint_raw/local', PoseStamped, queue_size=10, tracer_name='predictive_controller'
        )
        self.prediction_pub = create_traced_publisher(
            '/predicted_trajectory', Path, queue_size=10, tracer_name='predictive_controller'
        )
        self.optimization_pub = create_traced_publisher(
            '/optimization_results', String, queue_size=10, tracer_name='predictive_controller'
        )
        self.controller_status_pub = create_traced_publisher(
            '/controller_status', String, queue_size=10, tracer_name='predictive_controller'
        )

        # Timers
        self.prediction_timer = rospy.Timer(rospy.Duration(1.0 / self.control_frequency), self.prediction_loop)
        self.optimization_timer = rospy.Timer(rospy.Duration(1.0), self.optimization_loop)
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)

        # Attempt to bring up the neural network path. This only succeeds
        # if TF imported cleanly AND trained weights are found on disk.
        if self.neural_network_requested:
            self.initialize_neural_network()

        if not self.neural_network_enabled:
            rospy.loginfo(
                "Predictive Controller running WITHOUT a trained neural network "
                "-- using constant-velocity target prediction instead."
            )

        self.logger.info("node_initialized", extra={
            "horizon_length": self.horizon_length,
            "control_frequency": self.control_frequency,
            "neural_network_enabled": self.neural_network_enabled,
            "simulation_mode": self.simulation_mode
        })
        rospy.loginfo("Predictive Controller initialized")

    def _init_metrics(self):
        """Initialize Prometheus metrics for this node"""
        self.logger.debug("initializing_metrics")

        # Counters
        self.metric_predictions_total = self.metrics.counter(
            "predictions_total",
            "Total number of target-trajectory predictions run",
            labels=["method"]
        )
        self.metric_neural_prediction_failures_total = self.metrics.counter(
            "neural_prediction_failures_total",
            "Total number of neural-network predictions that raised an exception and fell back"
        )
        self.metric_emergency_stops_total = self.metrics.counter(
            "emergency_stops_total",
            "Total number of emergency stops triggered"
        )
        self.metric_errors_total = self.metrics.counter(
            "errors_total",
            "Total number of errors encountered",
            labels=["error_type"]
        )

        # Gauges
        self.metric_trajectory_quality = self.metrics.gauge(
            "trajectory_quality",
            "Current trajectory quality score (0-1)"
        )
        self.metric_prediction_error = self.metrics.gauge(
            "prediction_error",
            "Mean recent prediction error (position, meters)"
        )
        self.metric_control_sequence_length = self.metrics.gauge(
            "control_sequence_length",
            "Number of control commands remaining in the current sequence"
        )
        self.metric_neural_network_enabled = self.metrics.gauge(
            "neural_network_enabled",
            "Whether the neural prediction path is active (1=yes, 0=no)"
        )
        self.metric_emergency_stop_status = self.metrics.gauge(
            "emergency_stop_active",
            "Whether the controller is currently in emergency stop (1=yes, 0=no)"
        )

        # Histograms
        self.metric_prediction_loop_duration = self.metrics.histogram(
            "prediction_loop_duration_seconds",
            "Time taken by a full prediction_loop cycle",
            buckets=[0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5]
        )

        self.metric_neural_network_enabled.set(0)
        self.logger.info("metrics_initialized")

    def initialize_neural_network(self):
        """Load a trained neural network for trajectory prediction, if available.

        This intentionally does NOT build-and-use an untrained model. An
        untrained Sequential network's .predict() output is random noise
        (see module docstring) -- that is worse than no network at all,
        because it looks like a real prediction. We only enable the neural
        path if we can load real weights from disk.
        """
        if not TENSORFLOW_AVAILABLE:
            rospy.logwarn(
                "neural_network_enabled was requested but TensorFlow is not "
                "usable in this environment (import failed, e.g. AVX "
                "instructions unavailable on this CPU/VM). Falling back to "
                "constant-velocity prediction. See ~model_weights_path and "
                "the training plan for how to bring the neural path up."
            )
            self.logger.warning("tensorflow_unavailable")
            return

        if not self.model_weights_path or not os.path.exists(self.model_weights_path):
            rospy.logwarn(
                f"neural_network_enabled was requested but no trained weights "
                f"found at ~model_weights_path='{self.model_weights_path}'. "
                f"Falling back to constant-velocity prediction. Train a model "
                f"offline first (see training plan) and point this param at "
                f"the resulting weights file."
            )
            self.logger.warning("no_trained_weights_found", extra={
                "model_weights_path": self.model_weights_path
            })
            return

        try:
            self.neural_network_model = tf.keras.models.load_model(self.model_weights_path)
            self.neural_network_enabled = True
            self.metric_neural_network_enabled.set(1)
            rospy.loginfo(f"Loaded trained neural network from {self.model_weights_path}")
            self.logger.info("neural_network_loaded", extra={"path": self.model_weights_path})
        except Exception as e:
            rospy.logwarn(f"Could not load neural network weights: {e}")
            self.logger.exception("neural_network_load_failed", extra={"error": str(e)})
            self.metric_errors_total.labels(error_type="neural_network_load").inc()
            self.neural_network_model = None
            self.neural_network_enabled = False

    @traced_ros_callback('predictive_controller', 'odometry_callback')
    def odometry_callback(self, msg):
        """Update current vehicle state"""
        self.current_state = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])

        # Check for emergency stop
        if self.emergency_stop:
            self.trigger_emergency_stop()

    @traced_ros_callback('predictive_controller', 'tracked_targets_callback')
    def tracked_targets_callback(self, msg):
        """Handle tracked target updates"""
        if self.current_state is None:
            # Guard against target messages arriving before the first
            # odometry update -- avoids a crash on self.current_state[0].
            return

        if msg.targets:
            # Get the closest target
            closest_target = None
            min_distance = float('inf')

            for target in msg.targets:
                distance = np.linalg.norm([
                    target.position.x - self.current_state[0],
                    target.position.y - self.current_state[1],
                    target.position.z - self.current_state[2]
                ])

                if distance < min_distance:
                    min_distance = distance
                    closest_target = target

            if closest_target:
                # Predict target trajectory
                self.predict_target_trajectory(closest_target)

    @traced_ros_callback('predictive_controller', 'planner_status_callback')
    def planner_status_callback(self, msg):
        """Handle planner status updates.

        NOTE: trajectory_planner.publish_status() publishes plain strings
        (e.g. "emergency", "searching", "planning",
        "following_waypoints_2/5"), not JSON -- so the previous
        `json.loads(msg.data)` here always raised, was swallowed by the
        except block, and this callback never actually updated
        emergency_stop/status. Parse the plain-string protocol the planner
        actually uses instead.

        This also adds a recovery path: previously emergency_stop was only
        ever set to True and nothing ever cleared it, so a single emergency
        would latch the controller into PredictiveControllerStatus.EMERGENCY
        forever with no way back.
        """
        status = msg.data
        self.logger.debug("planner_status_received", extra={"status": status})

        if status in ('emergency', 'emergency_stop'):
            self.emergency_stop = True
            self.logger.warning("emergency_stop_set_from_planner", extra={"status": status})
        else:
            # Planner has reported a non-emergency status (searching,
            # planning, following_waypoints_*), so clear our own latched
            # emergency_stop and let the controller resume driving.
            if self.emergency_stop:
                self.logger.info("emergency_stop_cleared_from_planner", extra={"status": status})
            self.emergency_stop = False
            if self.status == PredictiveControllerStatus.EMERGENCY:
                self.status = PredictiveControllerStatus.IDLE

    def prediction_loop(self, event):
        """Main prediction loop"""
        with span_context(self.tracer, "prediction_loop") as span:
            with MetricTimer(self.metric_prediction_loop_duration):
                if self.emergency_stop:
                    span.set_attribute("skipped_reason", "emergency_stop")
                    return

                if self.current_state is not None:
                    self.status = PredictiveControllerStatus.PREDICTING

                    # Run prediction
                    self.run_prediction()

                    # Optimize controls
                    self.status = PredictiveControllerStatus.OPTIMIZING
                    self.optimize_controls()

                    # Execute optimized controls
                    self.status = PredictiveControllerStatus.CONTROLLING
                    self.execute_controls()

                    # Used by collect_performance_metrics() to report a real
                    # execution_time; previously this was never assigned after
                    # init, so execution_time = time.time() - 0.0 reported a
                    # Unix-timestamp-sized number instead of a duration.
                    self.last_update_time = time.time()

    def optimization_loop(self, event):
        """Performance optimization and monitoring"""
        with span_context(self.tracer, "optimization_loop") as span:
            # Collect performance metrics
            self.collect_performance_metrics()

            # Update trajectory quality
            if self.optimization_result:
                self.update_trajectory_quality()

            self.metric_trajectory_quality.set(self.trajectory_quality)
            self.metric_prediction_error.set(self.calculate_prediction_error())
            self.metric_control_sequence_length.set(len(self.control_sequence))
            span.set_attribute("trajectory_quality", self.trajectory_quality)

    @traced('predictive_controller', 'predict_target_trajectory')
    def predict_target_trajectory(self, target):
        """Predict target trajectory, using the trained network if loaded, otherwise the constant-velocity fallback"""
        if not self.neural_network_enabled:
            self.metric_predictions_total.labels(method="simple").inc()
            self.simple_target_prediction(target)
            return

        try:
            # Prepare input for neural network
            target_state = np.array([
                target.position.x,
                target.position.y,
                target.position.z,
                0.0, 0.0, 0.0,  # velocity
                self.current_state[0], self.current_state[1], self.current_state[2],  # relative position
            ])

            # Get prediction from neural network
            prediction = self.neural_network_model.predict(
                target_state.reshape(1, 9),
                verbose=0
            )

            # Extract predicted trajectory
            predicted_trajectory = prediction[0]

            # Convert to trajectory format
            self.target_trajectory = []
            for i in range(min(self.horizon_length, len(predicted_trajectory) // 3)):
                pos = predicted_trajectory[i*3:(i+1)*3]
                self.target_trajectory.append(pos)

            self.metric_predictions_total.labels(method="neural").inc()
            rospy.logdebug(f"Target trajectory predicted with {len(self.target_trajectory)} points")
            self.logger.debug("neural_target_trajectory_predicted", extra={
                "point_count": len(self.target_trajectory)
            })
        except Exception as e:
            rospy.logwarn(f"Neural network prediction failed: {e}")
            self.logger.exception("neural_prediction_failed", extra={"error": str(e)})
            self.metric_neural_prediction_failures_total.inc()
            self.metric_errors_total.labels(error_type="neural_prediction").inc()
            self.simple_target_prediction(target)

    def simple_target_prediction(self, target):
        """Constant-velocity target prediction (default / fallback path)"""
        target_velocity = np.array([
            target.velocity.x if hasattr(target, 'velocity') else 0.0,
            target.velocity.y if hasattr(target, 'velocity') else 0.0,
            target.velocity.z if hasattr(target, 'velocity') else 0.0
        ])

        # Generate predicted trajectory
        self.target_trajectory = []
        for i in range(self.horizon_length):
            future_position = np.array([
                target.position.x + target_velocity[0] * i * self.time_step,
                target.position.y + target_velocity[1] * i * self.time_step,
                target.position.z + target_velocity[2] * i * self.time_step
            ])

            # Add simulated uncertainty. This is only meaningful for
            # testing against a simulator -- on real hardware there's no
            # reason to inject fake noise into a real target's predicted
            # position, since it just makes tracking jitter for no benefit.
            if self.simulation_mode:
                future_position += np.random.normal(0, 0.1, 3)

            self.target_trajectory.append(future_position)

    @traced('predictive_controller', 'run_prediction')
    def run_prediction(self):
        """Run MPC prediction"""
        if self.current_state is None:
            return

        # Evaluate the *previous* cycle's first predicted future state
        # against where we actually ended up, so calculate_prediction_error()
        # / update_trajectory_quality() reflect real prediction accuracy
        # instead of staying at zero forever (prediction_errors was
        # previously never appended to anywhere in this file).
        if self.prediction_horizon and len(self.prediction_horizon) > 1:
            previously_predicted_next_state = self.prediction_horizon[1]
            position_error = float(np.linalg.norm(
                previously_predicted_next_state[0:3] - self.current_state[0:3]
            ))
            self.prediction_errors.append(position_error)
            if len(self.prediction_errors) > 100:
                self.prediction_errors.pop(0)

        # Predict future states based on current dynamics
        self.prediction_horizon = []

        # Initialize with current state
        current_state = self.current_state.copy()
        self.prediction_horizon.append(current_state)

        # Generate prediction horizon
        for step in range(self.horizon_length):
            # Predict next state
            future_state = self.predict_next_state(current_state)

            # Add prediction error to simulate uncertainty. As with the
            # target-prediction noise above, this should only apply when
            # testing against a simulator -- injecting random noise into
            # the state used by the control law on real hardware would
            # make every command noisier for no reason.
            if self.simulation_mode:
                future_state += self.add_prediction_error()

            # Add to horizon
            self.prediction_horizon.append(future_state)

            # Update current state for next iteration
            current_state = future_state

    def predict_next_state(self, state):
        """Predict next state using vehicle dynamics"""
        # Extract state variables
        position = state[0:3]
        velocity = state[3:6]
        angular = state[6:9]

        # Apply simple dynamics (constant acceleration)
        acceleration = np.array([0.0, 0.0, 0.0])  # Could be replaced with actual control

        # Predict next state
        next_position = position + velocity * self.time_step + 0.5 * acceleration * self.time_step**2
        next_velocity = velocity + acceleration * self.time_step
        next_angular = angular  # Assuming constant angular velocity

        # Combine into next state
        next_state = np.concatenate([next_position, next_velocity, next_angular])

        return next_state

    def add_prediction_error(self):
        """Add prediction error to simulate uncertainty"""
        # Gaussian noise for prediction uncertainty
        position_error = np.random.normal(0, 0.05, 3)  # 5 cm position uncertainty
        velocity_error = np.random.normal(0, 0.02, 3)  # 2 cm/s velocity uncertainty
        angular_error = np.random.normal(0, 0.01, 3)  # 0.01 rad/s angular uncertainty

        return np.concatenate([position_error, velocity_error, angular_error])

    def optimize_controls(self):
        """Optimize control sequence using MPC"""
        if self.prediction_horizon is None or len(self.prediction_horizon) == 0:
            return

        # Simple MPC optimization
        self.control_sequence = []

        # For each time step in horizon
        for step in range(self.horizon_length):
            # Calculate desired control based on target trajectory
            if self.target_trajectory and step < len(self.target_trajectory):
                desired_state = self.target_trajectory[step]
            else:
                desired_state = np.zeros(9)

            # Calculate control command
            control = self.calculate_control_command(self.prediction_horizon[step], desired_state)
            self.control_sequence.append(control)

        # Optimize with constraints
        self.optimize_with_constraints()

        rospy.logdebug(f"Control sequence optimized with {len(self.control_sequence)} steps")

    def calculate_control_command(self, current_state, desired_state):
        """Calculate control command to reach desired state"""
        # desired_state may be a 3-vector (from target_trajectory, position
        # only) or a 9-vector fallback (np.zeros(9)); normalize to avoid an
        # index error when slicing desired_state[3:6] on a length-3 array.
        desired_state = np.asarray(desired_state)
        if desired_state.shape[0] < 9:
            padded = np.zeros(9)
            padded[:desired_state.shape[0]] = desired_state
            desired_state = padded

        # Calculate position error
        position_error = desired_state[0:3] - current_state[0:3]
        velocity_error = desired_state[3:6] - current_state[3:6]

        # Calculate desired velocity
        desired_velocity = self.calculate_desired_velocity(position_error)

        # Create control command
        control = {
            'velocity': desired_velocity,
            'acceleration': velocity_error / self.time_step,
            'jerk': np.zeros(3),  # Could be calculated
            'smoothness': 1.0
        }

        return control

    def calculate_desired_velocity(self, position_error):
        """Calculate desired velocity based on position error"""
        distance = np.linalg.norm(position_error)

        if distance < self.safety_distance:
            # Stop or slow down if too close
            return np.zeros(3)

        # Calculate velocity
        velocity_magnitude = min(self.max_velocity, distance * 0.5)
        velocity_direction = position_error / (distance + 1e-6)

        desired_velocity = velocity_magnitude * velocity_direction

        return desired_velocity

    def optimize_with_constraints(self):
        """Apply constraints to control sequence"""
        # Apply velocity constraints
        for i, control in enumerate(self.control_sequence):
            velocity = control['velocity']
            velocity_magnitude = np.linalg.norm(velocity)

            if velocity_magnitude > self.max_velocity:
                control['velocity'] = velocity * (self.max_velocity / velocity_magnitude)

            # Apply acceleration constraints
            acceleration = control['acceleration']
            acceleration_magnitude = np.linalg.norm(acceleration)

            if acceleration_magnitude > self.max_acceleration:
                control['acceleration'] = acceleration * (self.max_acceleration / acceleration_magnitude)

    @traced('predictive_controller', 'execute_controls')
    def execute_controls(self):
        """Execute optimized control commands"""
        if not self.control_sequence or len(self.control_sequence) == 0:
            return

        # Get current control command
        current_control = self.control_sequence[0]

        # Execute velocity command
        twist = Twist()
        twist.linear.x = current_control['velocity'][0]
        twist.linear.y = current_control['velocity'][1]
        twist.linear.z = current_control['velocity'][2]

        # Publish control command
        self.control_pub.publish(twist)

        # Also publish a position setpoint for more precise control.
        # prediction_horizon[0] is a copy of the *current* actual state
        # (see run_prediction()), not a forward prediction -- publishing
        # it as the position setpoint told the flight controller to stay
        # exactly where it already was, fighting the velocity command
        # above and preventing any net movement toward the target. Use
        # the first predicted *future* state instead.
        if len(self.prediction_horizon) > 1:
            next_state = self.prediction_horizon[1]
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = next_state[0]
            pose.pose.position.y = next_state[1]
            pose.pose.position.z = next_state[2]
            self.position_target_pub.publish(pose)

        # Remove executed command
        self.control_sequence.pop(0)

        # Publish prediction trajectory
        self.publish_prediction_trajectory()

    def publish_prediction_trajectory(self):
        """Publish predicted trajectory"""
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"

        for state in self.prediction_horizon:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = state[0]
            pose.pose.position.y = state[1]
            pose.pose.position.z = state[2]
            path.poses.append(pose)

        self.prediction_pub.publish(path)

    def collect_performance_metrics(self):
        """Collect performance metrics"""
        metrics = {
            'prediction_error': self.calculate_prediction_error(),
            'control_smoothness': self.calculate_control_smoothness(),
            'trajectory_quality': self.trajectory_quality,
            'execution_time': time.time() - self.last_update_time if self.last_update_time else 0.0
        }

        self.control_performance.append(metrics)

        # Keep only recent metrics
        if len(self.control_performance) > 100:
            self.control_performance.pop(0)

    def calculate_prediction_error(self):
        """Calculate prediction error"""
        if not self.prediction_errors:
            return 0.0

        return np.mean(self.prediction_errors)

    def calculate_control_smoothness(self):
        """Calculate control smoothness"""
        if len(self.control_sequence) < 2:
            return 1.0

        smoothness = 0.0
        for i in range(1, len(self.control_sequence)):
            current = self.control_sequence[i-1]['velocity']
            next_control = self.control_sequence[i]['velocity']  # was indexing i-1 twice, comparing a value to itself

            difference = np.linalg.norm(next_control - current)
            smoothness += 1.0 / (difference + 1e-6)

        return smoothness / (len(self.control_sequence) - 1)

    def update_trajectory_quality(self):
        """Update trajectory quality metric"""
        if not self.prediction_errors:
            return

        # Calculate quality based on prediction errors
        mean_error = np.mean(self.prediction_errors)

        # Normalize quality between 0 and 1
        self.trajectory_quality = max(0.0, min(1.0, 1.0 - mean_error / 1.0))

    @traced('predictive_controller', 'trigger_emergency_stop')
    def trigger_emergency_stop(self):
        """Trigger emergency stop procedures"""
        rospy.logerr("Emergency stop triggered in predictive controller!")
        self.logger.error("emergency_stop_triggered")
        self.metric_emergency_stops_total.inc()

        # Stop all controls
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        self.control_pub.publish(twist)

        # Update status
        self.status = PredictiveControllerStatus.EMERGENCY

        # Publish emergency status
        status_msg = String()
        status_msg.data = json.dumps({
            'status': 'emergency',
            'reason': 'emergency_stop_triggered'
        })
        self.controller_status_pub.publish(status_msg)

    def publish_status(self, event):
        """Publish controller status"""
        with span_context(self.tracer, "publish_status") as span:
            status_msg = String()

            status_info = {
                'status': self.status.value,
                'prediction_horizon': len(self.prediction_horizon) if self.prediction_horizon else 0,
                'control_sequence_length': len(self.control_sequence),
                'target_trajectory_length': len(self.target_trajectory) if self.target_trajectory else 0,
                'trajectory_quality': self.trajectory_quality,
                'emergency_stop': self.emergency_stop,
                'neural_network_enabled': self.neural_network_enabled
            }

            status_msg.data = json.dumps(status_info)
            self.controller_status_pub.publish(status_msg)

            self.metric_emergency_stop_status.set(1 if self.emergency_stop else 0)
            span.set_attribute("status", self.status.value)
            span.set_attribute("emergency_stop", self.emergency_stop)
            self.logger.debug("status_published", extra=status_info)

if __name__ == '__main__':
    try:
        controller = PredictiveController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass