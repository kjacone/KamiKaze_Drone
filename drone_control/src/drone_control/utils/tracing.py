#!/usr/bin/env python3
"""
drone_control/scripts/utils/tracing.py
OpenTelemetry distributed tracing integration
"""

import os
import sys
from contextlib import contextmanager
from functools import wraps
from typing import Any, Dict

# Add parent directory to path
# sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from drone_control.utils.logging_framework import get_logger
except ImportError:
    import logging
    def get_logger(name):
        return logging.getLogger(name)

# OpenTelemetry imports
try:
    from opentelemetry import context, trace
    from opentelemetry.exporter.jaeger.thrift import JaegerExporter
    from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter
    from opentelemetry.instrumentation.grpc import GrpcInstrumentorClient
    from opentelemetry.instrumentation.requests import RequestsInstrumentor
    from opentelemetry.sdk.resources import Resource
    from opentelemetry.sdk.trace import TracerProvider
    from opentelemetry.sdk.trace.export import (
        BatchSpanProcessor,
        ConsoleSpanExporter,
    )
    from opentelemetry.trace import Status, StatusCode
    OTEL_AVAILABLE = True
except ImportError:
    OTEL_AVAILABLE = False
    class Status:
        def __init__(self, *args, **kwargs): pass
    class StatusCode:
        OK = 0
        ERROR = 1

    class Span:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def set_attribute(self, key, value): pass
        def set_status(self, status): pass
        def record_exception(self, exc): pass
        def end(self): pass

    class Tracer:
        def start_span(self, name, context=None, kind=None, attributes=None):
            return Span()

    class _DummyContext:
        @staticmethod
        def attach(*args, **kwargs):
            return None

        @staticmethod
        def detach(*args, **kwargs):
            pass

    context = _DummyContext()

    class _DummyTraceModule:
        @staticmethod
        def set_span_in_context(span):
            return None

        @staticmethod
        def use_span(span, end_on_exit=False):
            class _NullCtx:
                def __enter__(self): return None
                def __exit__(self, *args): pass
            return _NullCtx()

    trace = _DummyTraceModule()


# Initialize OpenTelemetry
_initialized = False
_logger = None


def init_tracing(
    service_name: str = "kamikaze-drone",
    jaeger_host: str = "jaeger",
    jaeger_port: int = 14250,
    use_otlp: bool = True,
    export_to_console: bool = False,
    component: str = None
):
    """
    Initialize OpenTelemetry tracing.

    Args:
        service_name: Name of the service
        jaeger_host: Jaeger collector host
        jaeger_port: Jaeger collector port
        use_otlp: Use OTLP exporter (gRPC) instead of Thrift
        export_to_console: Export spans to console for debugging
        component: Component name for logging
    """
    global _initialized, _logger

    if not OTEL_AVAILABLE:
        _logger = get_logger("tracing")
        _logger.warning("opentelemetry_not_available", extra={
            "message": "Install: pip install opentelemetry-api opentelemetry-sdk opentelemetry-exporter-jaeger opentelemetry-exporter-otlp"
        })
        return False

    if _initialized:
        return True

    # Get logger
    _logger = get_logger(component or "tracing")

    try:
        # Create resource with service information
        resource = Resource.create({
            "service.name": service_name,
            "service.version": "1.0.0",
            "deployment.environment": os.environ.get("ENVIRONMENT", "development"),
            "container.name": os.environ.get("HOSTNAME", "unknown"),
        })

        # Create tracer provider
        provider = TracerProvider(resource=resource)

        # Configure exporter
        if use_otlp:
            # OTLP gRPC exporter (recommended)
            exporter = OTLPSpanExporter(
                endpoint=f"{jaeger_host}:4317",
                insecure=True,
            )
            _logger.info("otel_exporter_configured", extra={
                "type": "otlp_grpc",
                "endpoint": f"{jaeger_host}:4317"
            })
        else:
            # Jaeger Thrift exporter (legacy)
            exporter = JaegerExporter(
                collector_endpoint=f"http://{jaeger_host}:14268/api/traces",
                agent_host_name=jaeger_host,
                agent_port=6831,
            )
            _logger.info("otel_exporter_configured", extra={
                "type": "jaeger_thrift",
                "endpoint": f"http://{jaeger_host}:14268/api/traces"
            })

        # Add batch processor
        span_processor = BatchSpanProcessor(exporter)
        provider.add_span_processor(span_processor)

        # Add console exporter for debugging
        if export_to_console:
            console_exporter = ConsoleSpanExporter()
            console_processor = BatchSpanProcessor(console_exporter)
            provider.add_span_processor(console_processor)
            _logger.info("otel_console_exporter_enabled")

        # Set global tracer provider
        trace.set_tracer_provider(provider)

        # Instrument common libraries
        try:
            RequestsInstrumentor().instrument()
        except Exception as e:
            _logger.debug("requests_instrumentation_failed", extra={"error": str(e)})

        try:
            GrpcInstrumentorClient().instrument()
        except Exception as e:
            _logger.debug("grpc_instrumentation_failed", extra={"error": str(e)})

        _initialized = True
        _logger.info("tracing_initialized", extra={
            "service_name": service_name,
            "jaeger_host": jaeger_host,
            "use_otlp": use_otlp
        })
        return True

    except Exception as e:
        _logger.error("tracing_initialization_failed", extra={
            "error": str(e)
        })
        return False


def get_tracer(name: str, version: str = "1.0.0") -> Tracer:
    """
    Get a tracer instance.

    Args:
        name: Tracer name (usually component name)
        version: Tracer version

    Returns:
        Tracer instance
    """
    if not OTEL_AVAILABLE:
        return Tracer()

    return trace.get_tracer(name, version)


class TracingContext:
    """Context manager for trace context propagation"""

    def __init__(self, tracer: Tracer, name: str, attributes: Dict[str, Any] = None):
        self.tracer = tracer
        self.name = name
        self.attributes = attributes or {}
        self.span = None
        self._token = None

    def __enter__(self):
        if OTEL_AVAILABLE and self.tracer:
            self.span = self.tracer.start_span(
                self.name,
                attributes=self.attributes
            )
            # Actually activate the span in the current context so that any
            # child spans created inside this `with` block are correctly
            # nested underneath it.
            self._token = context.attach(trace.set_span_in_context(self.span))
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.span:
            if exc_type:
                self.span.set_status(Status(StatusCode.ERROR, str(exc_val)))
                self.span.record_exception(exc_val)
            self.span.end()
        if self._token is not None:
            context.detach(self._token)


@contextmanager
def span_context(tracer: Tracer, name: str, attributes: Dict[str, Any] = None):
    """
    Context manager for creating spans.

    Usage:
        with span_context(tracer, "detect_objects", {"frame_id": 42}) as span:
            # do work
            span.set_attribute("detections", 3)

    Args:
        tracer: Tracer instance
        name: Span name
        attributes: Span attributes

    Yields:
        Span instance
    """
    if not OTEL_AVAILABLE or not tracer:
        class DummySpan:
            def set_attribute(self, key, value): pass
            def set_status(self, status): pass
            def record_exception(self, exc): pass
        yield DummySpan()
        return

    span = tracer.start_span(name, attributes=attributes)
    try:
        yield span
        span.set_status(Status(StatusCode.OK))
    except Exception as e:
        span.set_status(Status(StatusCode.ERROR, str(e)))
        span.record_exception(e)
        raise
    finally:
        span.end()


def traced(tracer_name: str, span_name: str = None, attributes: Dict[str, Any] = None):
    """
    Decorator for tracing functions.

    Usage:
        @traced("mission_manager", "execute_mission")
        def execute_mission(self, mission_id):
            # function body

    Args:
        tracer_name: Name for the tracer
        span_name: Name for the span (defaults to function name)
        attributes: Static attributes for the span
    """
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            tracer = get_tracer(tracer_name)
            span_name_ = span_name or func.__name__

            # Try to extract mission_id from args/kwargs
            mission_id = kwargs.get('mission_id')
            if not mission_id and args:
                # Check if first arg is self and has mission_id
                if len(args) > 0 and hasattr(args[0], 'mission_id'):
                    mission_id = args[0].mission_id
                elif len(args) > 0 and isinstance(args[0], str):
                    mission_id = args[0]
                elif len(args) > 1 and isinstance(args[1], str):
                    mission_id = args[1]

            span_attrs = attributes or {}
            if mission_id:
                span_attrs['mission_id'] = mission_id

            with span_context(tracer, span_name_, span_attrs) as span:
                # Add arguments as attributes (limited)
                if args and len(args) <= 3:
                    for i, arg in enumerate(args):
                        if isinstance(arg, (str, int, float, bool)):
                            span.set_attribute(f"arg_{i}", str(arg))

                # Add keyword arguments as attributes
                for key, value in kwargs.items():
                    if isinstance(value, (str, int, float, bool)):
                        span.set_attribute(f"arg_{key}", str(value))

                result = func(*args, **kwargs)

                # Add result info if possible
                if result is not None:
                    if isinstance(result, (str, int, float, bool)):
                        span.set_attribute("result", str(result))

                return result

        return wrapper

    return decorator


def add_span_attributes(span, attributes: Dict[str, Any]):
    """
    Add attributes to a span.

    Args:
        span: Span instance
        attributes: Dictionary of attributes
    """
    if not OTEL_AVAILABLE or not span:
        return

    for key, value in attributes.items():
        if isinstance(value, (str, int, float, bool)):
            span.set_attribute(key, value)
        elif isinstance(value, dict):
            # Flatten nested dict
            for k, v in value.items():
                if isinstance(v, (str, int, float, bool)):
                    span.set_attribute(f"{key}.{k}", v)


# Global tracer instances
_tracers = {}


def get_component_tracer(component: str, service_name: str = "kamikaze-drone") -> Tracer:
    """
    Get or create a tracer for a component.

    Args:
        component: Component name (e.g., "mission_manager")
        service_name: Service name

    Returns:
        Tracer instance
    """
    if component not in _tracers:
        _tracers[component] = get_tracer(f"{service_name}.{component}")
    return _tracers[component]


# Convenience function for instrumenting ROS nodes
def instrument_ros_node(node_name: str):
    """
    Instrument a ROS node with OpenTelemetry.

    Args:
        node_name: Name of the ROS node

    Returns:
        Tracer instance and a decorator for instrumentation
    """
    tracer = get_component_tracer(node_name)

    def ros_traced(span_name: str = None, attributes: Dict[str, Any] = None):
        return traced(node_name, span_name, attributes)

    return tracer, ros_traced