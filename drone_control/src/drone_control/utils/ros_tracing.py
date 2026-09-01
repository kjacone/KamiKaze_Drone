#!/usr/bin/env python3
"""
drone_control/scripts/utils/ros_tracing.py
ROS-specific tracing utilities
"""

import time
from functools import wraps
from typing import Callable

import rospy

try:
    from drone_control.utils.tracing import (
        OTEL_AVAILABLE,
        Status,
        StatusCode,
        get_component_tracer,
        span_context,
    )
except ImportError:
    OTEL_AVAILABLE = False

    class StatusCode:
        OK = 0
        ERROR = 1

    class Status:
        def __init__(self, *args, **kwargs):
            pass

    class DummySpan:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def set_attribute(self, *args): pass
        def set_status(self, *args): pass
        def record_exception(self, *args): pass

    class DummyTracer:
        def start_span(self, *args, **kwargs):
            return DummySpan()

    def get_component_tracer(name):
        return DummyTracer()

    def span_context(*args, **kwargs):
        return DummySpan()


def traced_ros_callback(tracer_name: str, span_name: str = None):
    """
    Decorator for ROS callbacks with tracing.

    Usage:
        @traced_ros_callback("mission_manager", "command_callback")
        def _command_callback(self, msg):
            # callback body
    """
    def decorator(func):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            if not OTEL_AVAILABLE:
                return func(self, *args, **kwargs)

            tracer = get_component_tracer(tracer_name)
            span_name_ = span_name or func.__name__

            # Get mission_id from message if available
            mission_id = None
            for arg in args:
                if hasattr(arg, 'mission_id'):
                    mission_id = arg.mission_id
                    break

            with span_context(tracer, span_name_, {
                "mission_id": mission_id or "unknown",
                "callback": func.__name__
            }) as span:
                # Add message type if available
                if args and len(args) > 0:
                    msg = args[0]
                    if hasattr(msg, '__class__'):
                        span.set_attribute("message_type", msg.__class__.__name__)

                result = func(self, *args, **kwargs)
                return result

        return wrapper
    return decorator


class TracedService:
    """Wrapper for ROS services with tracing"""

    def __init__(self, service_name: str, service_type, handler: Callable,
                 tracer_name: str = None):
        """
        Create a traced ROS service.

        Args:
            service_name: Service name
            service_type: Service type class
            handler: Service handler function
            tracer_name: Tracer name (defaults to service_name)
        """
        self.service_name = service_name
        self.tracer_name = tracer_name or service_name.replace('/', '_')
        self.tracer = get_component_tracer(self.tracer_name)
        self.handler = handler
        self.service = rospy.Service(service_name, service_type, self._traced_handler)

    def _traced_handler(self, req):
        """Traced service handler"""
        with span_context(self.tracer, f"service.{self.service_name}", {
            "service_name": self.service_name,
            "request_type": req.__class__.__name__
        }) as span:
            try:
                start_time = time.time()
                result = self.handler(req)
                span.set_attribute("duration_seconds", time.time() - start_time)
                span.set_attribute("success", True)
                return result
            except Exception as e:
                span.set_status(Status(StatusCode.ERROR, str(e)))
                span.record_exception(e)
                raise


def create_traced_publisher(topic: str, msg_type, queue_size: int = 10,
                           tracer_name: str = None):
    """
    Create a traced ROS publisher.

    Args:
        topic: Topic name
        msg_type: Message type
        queue_size: Queue size
        tracer_name: Tracer name

    Returns:
        TracedPublisher instance
    """
    tracer = get_component_tracer(tracer_name or topic.replace('/', '_'))
    return TracedPublisher(topic, msg_type, queue_size, tracer)


class TracedPublisher:
    """ROS publisher with tracing"""

    def __init__(self, topic: str, msg_type, queue_size: int, tracer):
        self.topic = topic
        self.msg_type = msg_type
        self.tracer = tracer
        self.publisher = rospy.Publisher(topic, msg_type, queue_size=queue_size)

    def publish(self, msg):
        """Publish message with tracing"""
        with span_context(self.tracer, f"publish.{self.topic}", {
            "topic": self.topic,
            "message_type": msg.__class__.__name__
        }) as span:
            if hasattr(msg, 'mission_id'):
                span.set_attribute("mission_id", msg.mission_id)
            self.publisher.publish(msg)