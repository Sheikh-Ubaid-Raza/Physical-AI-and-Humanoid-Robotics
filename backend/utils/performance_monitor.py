"""
Response time monitoring utilities for the RAG Chatbot API
"""

import time
import statistics
from typing import Dict, List, Optional
from datetime import datetime, timedelta
from collections import defaultdict, deque
import threading


class ResponseTimeMonitor:
    """
    Monitor and track API response times for performance analysis
    """
    def __init__(self, window_size: int = 1000):  # Track last 1000 requests
        self.window_size = window_size
        self.response_times: Dict[str, deque] = defaultdict(lambda: deque(maxlen=self.window_size))
        self.lock = threading.Lock()

    def record_response_time(self, endpoint: str, response_time: float):
        """
        Record the response time for a specific endpoint
        """
        with self.lock:
            self.response_times[endpoint].append({
                'timestamp': datetime.now(),
                'response_time': response_time
            })

    def get_average_response_time(self, endpoint: str) -> Optional[float]:
        """
        Get the average response time for an endpoint
        """
        with self.lock:
            if endpoint in self.response_times and self.response_times[endpoint]:
                times = [record['response_time'] for record in self.response_times[endpoint]]
                return statistics.mean(times)
            return None

    def get_p95_response_time(self, endpoint: str) -> Optional[float]:
        """
        Get the 95th percentile response time for an endpoint
        """
        with self.lock:
            if endpoint in self.response_times and self.response_times[endpoint]:
                times = [record['response_time'] for record in self.response_times[endpoint]]
                if len(times) >= 20:  # Need at least 20 samples for meaningful percentile
                    sorted_times = sorted(times)
                    index = int(0.95 * len(sorted_times))
                    return sorted_times[index]
            return None

    def get_recent_response_times(self, endpoint: str, count: int = 10) -> List[float]:
        """
        Get the most recent response times for an endpoint
        """
        with self.lock:
            if endpoint in self.response_times:
                records = list(self.response_times[endpoint])
                recent_records = records[-count:] if len(records) >= count else records
                return [record['response_time'] for record in recent_records]
            return []

    def get_endpoint_stats(self, endpoint: str) -> Dict[str, Optional[float]]:
        """
        Get comprehensive stats for an endpoint
        """
        avg_time = self.get_average_response_time(endpoint)
        p95_time = self.get_p95_response_time(endpoint)
        recent_times = self.get_recent_response_times(endpoint, 10)

        return {
            'average_response_time': avg_time,
            'p95_response_time': p95_time,
            'recent_response_times': recent_times,
            'sample_count': len(self.response_times[endpoint]) if endpoint in self.response_times else 0
        }

    def is_slow_endpoint(self, endpoint: str, threshold: float = 5.0) -> bool:
        """
        Check if an endpoint is performing slowly (above threshold)
        """
        avg_time = self.get_average_response_time(endpoint)
        return avg_time is not None and avg_time > threshold


class PerformanceTracker:
    """
    Decorator and utility class to track performance of functions/endpoints
    """
    def __init__(self):
        self.monitor = ResponseTimeMonitor()

    def track_performance(self, endpoint_name: str):
        """
        Decorator to track performance of functions
        """
        def decorator(func):
            def wrapper(*args, **kwargs):
                start_time = time.time()
                try:
                    result = func(*args, **kwargs)
                    return result
                finally:
                    end_time = time.time()
                    response_time = end_time - start_time
                    self.monitor.record_response_time(endpoint_name, response_time)
            return wrapper
        return decorator

    def time_block(self, endpoint_name: str):
        """
        Context manager to time a block of code
        """
        class Timer:
            def __enter__(self):
                self.start_time = time.time()
                return self

            def __exit__(self, type, value, traceback):
                end_time = time.time()
                response_time = end_time - self.start_time
                self.monitor.record_response_time(endpoint_name, response_time)

        return Timer()

    def get_overall_performance_summary(self) -> Dict[str, Dict[str, Optional[float]]]:
        """
        Get performance summary for all tracked endpoints
        """
        summary = {}
        for endpoint in self.monitor.response_times.keys():
            summary[endpoint] = self.monitor.get_endpoint_stats(endpoint)
        return summary

    def get_slow_endpoints(self, threshold: float = 5.0) -> List[str]:
        """
        Get list of endpoints performing above the threshold
        """
        slow_endpoints = []
        for endpoint in self.monitor.response_times.keys():
            if self.monitor.is_slow_endpoint(endpoint, threshold):
                slow_endpoints.append(endpoint)
        return slow_endpoints


class PerformanceMetricsMiddleware:
    """
    FastAPI middleware to track response times for all endpoints
    """
    def __init__(self, app, performance_tracker: PerformanceTracker):
        self.app = app
        self.performance_tracker = performance_tracker

    async def __call__(self, scope, receive, send):
        if scope["type"] != "http":
            return await self.app(scope, receive, send)

        # Get the endpoint path
        path = scope.get("path", "unknown")
        method = scope.get("method", "UNKNOWN")

        # Create a wrapper for the send function to capture timing
        start_time = time.time()

        async def send_wrapper(message):
            if message["type"] == "http.response.start":
                # Calculate response time when response starts
                response_time = time.time() - start_time

                # Record performance metric
                endpoint_key = f"{method} {path}"
                self.performance_tracker.monitor.record_response_time(endpoint_key, response_time)

                # Add performance header to response if needed
                headers = message.get("headers", [])
                headers.append([b"x-response-time", f"{response_time:.3f}s".encode()])
                message["headers"] = headers

            await send(message)

        await self.app(scope, receive, send_wrapper)


# Global performance tracker instance
performance_tracker = PerformanceTracker()


def get_performance_tracker() -> PerformanceTracker:
    """
    Get the global performance tracker instance
    """
    return performance_tracker


def get_response_time_monitor() -> ResponseTimeMonitor:
    """
    Get the global response time monitor instance
    """
    return performance_tracker.monitor


# Utility function to time API calls
def time_api_call(endpoint: str, func, *args, **kwargs):
    """
    Time an API call and record the performance
    """
    start_time = time.time()
    try:
        result = func(*args, **kwargs)
        return result
    finally:
        end_time = time.time()
        response_time = end_time - start_time
        get_response_time_monitor().record_response_time(endpoint, response_time)


# Performance monitoring decorator
def monitor_performance(endpoint_name: str):
    """
    Decorator to monitor performance of API endpoints
    """
    tracker = get_performance_tracker()
    return tracker.track_performance(endpoint_name)


# Context manager for performance timing
def performance_timer(endpoint_name: str):
    """
    Context manager to time a specific operation
    """
    tracker = get_performance_tracker()
    return tracker.time_block(endpoint_name)