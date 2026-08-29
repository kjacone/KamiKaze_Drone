#!/usr/bin/env python3
"""
Neural Planner - Reinforcement Learning for Trajectory Optimization
Continuous learning from flight data with simulation-to-reality transfer
"""

import rospy
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from collections import deque
import random
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from drone_control.msg import ControlCommand, TrackedTargets
import threading

class NeuralPlanner(nn.Module):
    """Neural network for trajectory planning"""
    
    def __init__(self, state_dim=12, action_dim=4, hidden_dim=256):
        super(NeuralPlanner, self).__init__()
        
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, hidden_dim)
        self.fc4 = nn.Linear(hidden_dim, action_dim)
        
        self.dropout = nn.Dropout(0.1)
        
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = self.dropout(x)
        x = F.relu(self.fc2(x))
        x = self.dropout(x)
        x = F.relu(self.fc3(x))
        x = self.dropout(x)
        x = torch.tanh(self.fc4(x))  # Output in [-1, 1]
        return x

class ReplayBuffer:
    """Experience replay buffer for RL training"""
    
    def __init__(self, capacity=100000):
        self.buffer = deque(maxlen=capacity)
        self.lock = threading.Lock()
    
    def push(self, state, action, reward, next_state, done):
        with self.lock:
            self.buffer.append((state, action, reward, next_state, done))
    
    def sample(self, batch_size):
        with self.lock:
            batch = random.sample(self.buffer, min(batch_size, len(self.buffer)))
            return zip(*batch)
    
    def __len__(self):
        with self.lock:
            return len(self.buffer)

class NeuralPlannerNode:
    def __init__(self):
        rospy.init_node('neural_planner', anonymous=True)
        
        # Configuration
        self.state_dim = rospy.get_param('~state_dim', 12)
        self.action_dim = rospy.get_param('~action_dim', 4)
        self.hidden_dim = rospy.get_param('~hidden_dim', 256)
        self.learning_rate = rospy.get_param('~learning_rate', 1e-4)
        self.gamma = rospy.get_param('~gamma', 0.99)
        self.tau = rospy.get_param('~tau', 0.001)  # Target update rate
        self.buffer_size = rospy.get_param('~buffer_size', 100000)
        self.batch_size = rospy.get_param('~batch_size', 128)
        self.learning_enabled = rospy.get_param('~learning_enabled', True)
        
        # Initialize networks
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.policy_net = NeuralPlanner(self.state_dim, self.action_dim, self.hidden_dim).to(self.device)
        self.target_net = NeuralPlanner(self.state_dim, self.action_dim, self.hidden_dim).to(self.device)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        
        # Optimizer
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=self.learning_rate)
        
        # Replay buffer
        self.replay_buffer = ReplayBuffer(self.buffer_size)
        
        # State management
        self.current_state = None
        self.prev_state = None
        self.last_action = None
        
        # Setup subscribers
        self.setup_subscribers()
        
        # Setup publishers
        self.trajectory_pub = rospy.Publisher('/neural/trajectory', Path, queue_size=10)
        self.command_pub = rospy.Publisher('/neural/control_command', ControlCommand, queue_size=10)
        
        # Performance metrics
        self.rewards_history = deque(maxlen=100)
        self.loss_history = deque(maxlen=100)
        
        rospy.loginfo(f"Neural Planner initialized on {self.device}")
    
    def setup_subscribers(self):
        """Setup ROS subscribers"""
        # Drone state
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', 
                                        PoseStamped, self.pose_callback)
        
        # Velocity
        self.vel_sub = rospy.Subscriber('/mavros/local_position/velocity', 
                                       TwistStamped, self.vel_callback)
        
        # Target detection
        self.target_sub = rospy.Subscriber('/tracked_targets', 
                                          TrackedTargets, self.target_callback)
        
        # Store latest values
        self.latest_pose = None
        self.latest_vel = None
        self.latest_targets = None
    
    def pose_callback(self, msg):
        self.latest_pose = msg
    
    def vel_callback(self, msg):
        self.latest_vel = msg
    
    def target_callback(self, msg):
        self.latest_targets = msg
        if len(msg.targets) > 0:
            target = msg.targets[0]
            self.plan_trajectory(target)
    
    def get_state(self, target=None):
        """Get current state vector for neural network"""
        if self.latest_pose is None:
            return None
        
        state = []
        
        # Position (3)
        state.append(self.latest_pose.pose.position.x)
        state.append(self.latest_pose.pose.position.y)
        state.append(self.latest_pose.pose.position.z)
        
        # Orientation (quaternion)
        state.append(self.latest_pose.pose.orientation.x)
        state.append(self.latest_pose.pose.orientation.y)
        state.append(self.latest_pose.pose.orientation.z)
        state.append(self.latest_pose.pose.orientation.w)
        
        # Velocity (3)
        if self.latest_vel is not None:
            state.append(self.latest_vel.twist.linear.x)
            state.append(self.latest_vel.twist.linear.y)
            state.append(self.latest_vel.twist.linear.z)
        else:
            state.extend([0, 0, 0])
        
        # Target position (2) - relative
        if target is not None:
            dx = target.position.x - self.latest_pose.pose.position.x
            dy = target.position.y - self.latest_pose.pose.position.y
            dz = target.position.z - self.latest_pose.pose.position.z
            state.append(dx)
            state.append(dy)
            state.append(dz)
        else:
            state.extend([0, 0, 0])
        
        return np.array(state, dtype=np.float32)
    
    def plan_trajectory(self, target):
        """Plan trajectory using neural network"""
        state = self.get_state(target)
        if state is None:
            return
        
        # Convert to tensor
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        
        # Get action from policy network
        with torch.no_grad():
            action = self.policy_net(state_tensor).cpu().numpy()[0]
        
        # Scale action to physical control commands
        control = self.scale_action(action)
        
        # Publish control command
        self.publish_control(control)
        
        # Publish trajectory path
        self.publish_trajectory(target, control)
        
        # Store experience for learning
        if self.learning_enabled and self.prev_state is not None:
            reward = self.calculate_reward(state, target)
            self.replay_buffer.push(
                self.prev_state, self.last_action, reward, state, False
            )
            
            # Train network
            if len(self.replay_buffer) >= self.batch_size:
                self.train_network()
        
        # Update state
        self.prev_state = state.copy()
        self.last_action = action.copy()
    
    def scale_action(self, action):
        """Scale neural network output to physical control commands"""
        # action in [-1, 1]
        max_velocity = rospy.get_param('~max_velocity', 10.0)
        max_yaw_rate = rospy.get_param('~max_yaw_rate', 1.0)
        
        control = {
            'vx': action[0] * max_velocity,
            'vy': action[1] * max_velocity,
            'vz': action[2] * max_velocity,
            'yaw_rate': action[3] * max_yaw_rate,
            'thrust': rospy.get_param('~hover_thrust', 0.5) + action[3] * 0.2
        }
        return control
    
    def calculate_reward(self, state, target):
        """Calculate reward for current state"""
        reward = 0.0
        
        # Distance to target (negative)
        dx = state[10]  # Relative x
        dy = state[11]  # Relative y
        dz = state[12]  # Relative z
        distance = np.sqrt(dx**2 + dy**2 + dz**2)
        reward -= distance * 0.1
        
        # Velocity penalty (negative)
        vx, vy, vz = state[7], state[8], state[9]
        speed = np.sqrt(vx**2 + vy**2 + vz**2)
        reward -= speed * 0.01
        
        # Energy penalty (negative)
        thrust = rospy.get_param('~hover_thrust', 0.5)
        reward -= thrust * 0.001
        
        # Smoothness penalty (negative)
        if self.last_action is not None:
            action_diff = np.linalg.norm(self.last_action - state[:4])
            reward -= action_diff * 0.1
        
        return reward
    
    def train_network(self):
        """Train neural network using experience replay"""
        if len(self.replay_buffer) < self.batch_size:
            return
        
        # Sample batch
        states, actions, rewards, next_states, dones = self.replay_buffer.sample(self.batch_size)
        
        # Convert to tensors
        states = torch.FloatTensor(np.array(states)).to(self.device)
        actions = torch.FloatTensor(np.array(actions)).to(self.device)
        rewards = torch.FloatTensor(np.array(rewards)).unsqueeze(1).to(self.device)
        next_states = torch.FloatTensor(np.array(next_states)).to(self.device)
        dones = torch.FloatTensor(np.array(dones)).unsqueeze(1).to(self.device)
        
        # Compute Q values
        q_values = self.policy_net(states)
        next_q_values = self.target_net(next_states)
        
        # Compute targets
        targets = rewards + self.gamma * next_q_values.max(1, keepdim=True)[0] * (1 - dones)
        
        # Compute loss
        loss = F.mse_loss(q_values.gather(1, actions.long()), targets.detach())
        
        # Optimize
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
        # Update target network
        self.soft_update_target()
        
        # Store loss
        self.loss_history.append(loss.item())
    
    def soft_update_target(self):
        """Soft update target network"""
        for target_param, param in zip(self.target_net.parameters(), 
                                      self.policy_net.parameters()):
            target_param.data.copy_(
                self.tau * param.data + (1 - self.tau) * target_param.data
            )
    
    def publish_control(self, control):
        """Publish control command"""
        cmd = ControlCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.velocity.x = control['vx']
        cmd.velocity.y = control['vy']
        cmd.velocity.z = control['vz']
        cmd.yaw_rate = control['yaw_rate']
        cmd.thrust = control['thrust']
        self.command_pub.publish(cmd)
    
    def publish_trajectory(self, target, control):
        """Publish planned trajectory"""
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'world'
        
        # Generate waypoints for trajectory
        num_waypoints = 20
        dt = 0.1
        
        # Starting from current position
        start_pose = PoseStamped()
        start_pose.header = path.header
        start_pose.pose = self.latest_pose.pose
        path.poses.append(start_pose)
        
        # Simulate trajectory
        x, y, z = (self.latest_pose.pose.position.x, 
                   self.latest_pose.pose.position.y,
                   self.latest_pose.pose.position.z)
        
        vx, vy, vz = (control['vx'], control['vy'], control['vz'])
        
        for i in range(1, num_waypoints):
            pose = PoseStamped()
            pose.header = path.header
            
            # Update position
            x += vx * dt
            y += vy * dt
            z += vz * dt
            
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            
            # Keep orientation
            pose.pose.orientation = self.latest_pose.pose.orientation
            
            path.poses.append(pose)
        
        self.trajectory_pub.publish(path)
    
    def run(self):
        """Main loop"""
        rate = rospy.Rate(20)  # 20 Hz
        while not rospy.is_shutdown():
            # Print performance metrics
            if rospy.get_param('/debug_mode', False) and len(self.rewards_history) > 0:
                avg_reward = np.mean(self.rewards_history)
                avg_loss = np.mean(self.loss_history) if len(self.loss_history) > 0 else 0
                rospy.loginfo(f"Avg Reward: {avg_reward:.3f}, Avg Loss: {avg_loss:.6f}")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        planner = NeuralPlannerNode()
        planner.run()
    except rospy.ROSInterruptException:
        pass