"""
Performance monitoring dashboard for the RAG Chatbot API
"""

import time
import psutil
import os
from typing import Dict, Any, List
from datetime import datetime, timedelta
from collections import deque, defaultdict
import threading
import json
from fastapi import HTTPException
from fastapi.responses import JSONResponse


class PerformanceMonitor:
    """
    Monitor system and API performance metrics
    """
    def __init__(self, max_samples: int = 1000):
        self.max_samples = max_samples
        self.response_times: deque = deque(maxlen=max_samples)
        self.request_counts: deque = deque(maxlen=max_samples)
        self.error_counts: deque = deque(maxlen=max_samples)
        self.memory_usage: deque = deque(maxlen=max_samples)
        self.cpu_usage: deque = deque(maxlen=max_samples)
        self.lock = threading.Lock()

    def record_request(self, endpoint: str, response_time: float, is_error: bool = False):
        """
        Record a request with its response time and error status
        """
        with self.lock:
            self.response_times.append({
                'endpoint': endpoint,
                'response_time': response_time,
                'timestamp': datetime.now(),
                'is_error': is_error
            })

            # Update counters
            self.request_counts.append({
                'timestamp': datetime.now(),
                'count': 1
            })

            if is_error:
                self.error_counts.append({
                    'timestamp': datetime.now(),
                    'count': 1
                })

    def get_current_system_metrics(self) -> Dict[str, float]:
        """
        Get current system metrics (CPU, memory, disk usage)
        """
        return {
            'cpu_percent': psutil.cpu_percent(interval=1),
            'memory_percent': psutil.virtual_memory().percent,
            'disk_percent': psutil.disk_usage('/').percent,
            'memory_available_mb': psutil.virtual_memory().available / (1024 * 1024),
            'timestamp': datetime.now().isoformat()
        }

    def get_response_time_metrics(self, minutes: int = 5) -> Dict[str, Any]:
        """
        Get response time metrics for the last specified minutes
        """
        cutoff_time = datetime.now() - timedelta(minutes=minutes)

        with self.lock:
            recent_responses = [r for r in self.response_times if r['timestamp'] > cutoff_time]

            if not recent_responses:
                return {
                    'avg_response_time': 0,
                    'p95_response_time': 0,
                    'min_response_time': 0,
                    'max_response_time': 0,
                    'total_requests': 0,
                    'error_rate': 0
                }

            response_times = [r['response_time'] for r in recent_responses]
            total_requests = len(recent_responses)
            error_count = sum(1 for r in recent_responses if r['is_error'])

            # Calculate percentiles
            sorted_times = sorted(response_times)
            p95_idx = int(0.95 * len(sorted_times))
            p95_time = sorted_times[min(p95_idx, len(sorted_times) - 1)] if sorted_times else 0

            return {
                'avg_response_time': sum(response_times) / len(response_times),
                'p95_response_time': p95_time,
                'min_response_time': min(response_times),
                'max_response_time': max(response_times),
                'total_requests': total_requests,
                'error_rate': error_count / total_requests if total_requests > 0 else 0
            }

    def get_endpoint_metrics(self, minutes: int = 5) -> Dict[str, Dict[str, Any]]:
        """
        Get metrics grouped by endpoint
        """
        cutoff_time = datetime.now() - timedelta(minutes=minutes)

        with self.lock:
            endpoint_stats = defaultdict(list)

            for response in self.response_times:
                if response['timestamp'] > cutoff_time:
                    endpoint_stats[response['endpoint']].append(response)

            result = {}
            for endpoint, responses in endpoint_stats.items():
                response_times = [r['response_time'] for r in responses]
                error_count = sum(1 for r in responses if r['is_error'])

                if response_times:
                    result[endpoint] = {
                        'avg_response_time': sum(response_times) / len(response_times),
                        'min_response_time': min(response_times),
                        'max_response_time': max(response_times),
                        'total_requests': len(responses),
                        'error_rate': error_count / len(responses),
                        'last_seen': max(r['timestamp'] for r in responses).isoformat()
                    }

            return result

    def get_throughput_metrics(self, minutes: int = 5) -> Dict[str, Any]:
        """
        Get request throughput metrics
        """
        cutoff_time = datetime.now() - timedelta(minutes=minutes)

        with self.lock:
            recent_requests = [r for r in self.request_counts if r['timestamp'] > cutoff_time]

            if not recent_requests:
                return {
                    'requests_per_minute': 0,
                    'total_requests': 0
                }

            total_requests = sum(r['count'] for r in recent_requests)
            time_diff = minutes  # Since we're looking at last 'minutes' minutes

            return {
                'requests_per_minute': total_requests / time_diff if time_diff > 0 else 0,
                'total_requests': total_requests
            }

    def get_error_metrics(self, minutes: int = 5) -> Dict[str, Any]:
        """
        Get error rate metrics
        """
        cutoff_time = datetime.now() - timedelta(minutes=minutes)

        with self.lock:
            recent_errors = [e for e in self.error_counts if e['timestamp'] > cutoff_time]

            total_errors = sum(e['count'] for e in recent_errors)
            total_requests = len([r for r in self.response_times if r['timestamp'] > cutoff_time])

            error_rate = total_errors / total_requests if total_requests > 0 else 0

            return {
                'total_errors': total_errors,
                'error_rate': error_rate,
                'errors_per_minute': total_errors / minutes if minutes > 0 else 0
            }

    def get_performance_summary(self) -> Dict[str, Any]:
        """
        Get a comprehensive performance summary
        """
        return {
            'system_metrics': self.get_current_system_metrics(),
            'response_time_metrics': self.get_response_time_metrics(minutes=5),
            'throughput_metrics': self.get_throughput_metrics(minutes=5),
            'error_metrics': self.get_error_metrics(minutes=5),
            'endpoint_metrics': self.get_endpoint_metrics(minutes=5),
            'timestamp': datetime.now().isoformat()
        }


class PerformanceDashboard:
    """
    Dashboard for monitoring and displaying performance metrics
    """
    def __init__(self):
        self.monitor = PerformanceMonitor()
        self.metrics_history: deque = deque(maxlen=100)  # Keep last 100 snapshots

    def get_dashboard_data(self) -> Dict[str, Any]:
        """
        Get dashboard data for UI display
        """
        current_metrics = self.monitor.get_performance_summary()

        # Add to history
        self.metrics_history.append({
            'timestamp': current_metrics['timestamp'],
            'summary': current_metrics
        })

        return {
            'current_metrics': current_metrics,
            'historical_trend': list(self.metrics_history)[-10:],  # Last 10 snapshots
            'health_status': self._calculate_health_status(current_metrics)
        }

    def _calculate_health_status(self, metrics: Dict[str, Any]) -> str:
        """
        Calculate overall health status based on metrics
        """
        response_metrics = metrics['response_time_metrics']
        system_metrics = metrics['system_metrics']
        error_metrics = metrics['error_metrics']

        # Define health thresholds
        slow_response_threshold = 5.0  # seconds
        high_error_rate_threshold = 0.1  # 10%
        high_cpu_threshold = 80.0  # %
        high_memory_threshold = 80.0  # %

        issues = []

        if response_metrics['avg_response_time'] > slow_response_threshold:
            issues.append('slow_response_times')

        if error_metrics['error_rate'] > high_error_rate_threshold:
            issues.append('high_error_rate')

        if system_metrics['cpu_percent'] > high_cpu_threshold:
            issues.append('high_cpu_usage')

        if system_metrics['memory_percent'] > high_memory_threshold:
            issues.append('high_memory_usage')

        if not issues:
            return 'healthy'
        elif len(issues) == 1:
            return 'degraded'
        else:
            return 'unhealthy'

    def get_health_report(self) -> Dict[str, Any]:
        """
        Get a health report for the API
        """
        metrics = self.monitor.get_performance_summary()

        return {
            'status': self._calculate_health_status(metrics),
            'timestamp': datetime.now().isoformat(),
            'metrics': {
                'response_time': metrics['response_time_metrics'],
                'system': metrics['system_metrics'],
                'errors': metrics['error_metrics'],
                'throughput': metrics['throughput_metrics']
            },
            'recommendations': self._generate_recommendations(metrics)
        }

    def _generate_recommendations(self, metrics: Dict[str, Any]) -> List[str]:
        """
        Generate recommendations based on current metrics
        """
        recommendations = []

        response_metrics = metrics['response_time_metrics']
        system_metrics = metrics['system_metrics']
        error_metrics = metrics['error_metrics']

        if response_metrics['avg_response_time'] > 5.0:
            recommendations.append("Consider optimizing database queries or adding caching")

        if system_metrics['cpu_percent'] > 80.0:
            recommendations.append("High CPU usage detected - consider scaling or optimization")

        if system_metrics['memory_percent'] > 80.0:
            recommendations.append("High memory usage detected - investigate memory leaks or consider scaling")

        if error_metrics['error_rate'] > 0.05:  # 5% error rate
            recommendations.append("High error rate detected - investigate error logs")

        if not recommendations:
            recommendations.append("System is performing optimally")

        return recommendations

    def record_api_call(self, endpoint: str, start_time: float, is_error: bool = False):
        """
        Record an API call with its metrics
        """
        response_time = time.time() - start_time
        self.monitor.record_request(endpoint, response_time, is_error)


# Global performance dashboard instance
performance_dashboard = PerformanceDashboard()


def get_performance_dashboard() -> PerformanceDashboard:
    """
    Get the global performance dashboard instance
    """
    return performance_dashboard


# FastAPI middleware for performance monitoring
from fastapi import Request
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response


class PerformanceMonitoringMiddleware(BaseHTTPMiddleware):
    """
    Middleware to track performance metrics for all API endpoints
    """
    def __init__(self, app):
        super().__init__(app)
        self.dashboard = get_performance_dashboard()

    async def dispatch(self, request: Request, call_next):
        start_time = time.time()
        endpoint = f"{request.method} {request.url.path}"

        try:
            response = await call_next(request)
            # Record successful request
            self.dashboard.record_api_call(endpoint, start_time, is_error=False)
            return response
        except Exception as e:
            # Record failed request
            self.dashboard.record_api_call(endpoint, start_time, is_error=True)
            raise e


# API endpoints for performance metrics
from fastapi import APIRouter

router = APIRouter()


@router.get("/api/v1/performance/dashboard")
async def get_performance_dashboard_data():
    """
    Get comprehensive performance dashboard data
    """
    dashboard = get_performance_dashboard()
    return dashboard.get_dashboard_data()


@router.get("/api/v1/performance/health")
async def get_health_report():
    """
    Get health report with performance metrics
    """
    dashboard = get_performance_dashboard()
    return dashboard.get_health_report()


@router.get("/api/v1/performance/metrics")
async def get_detailed_metrics():
    """
    Get detailed performance metrics
    """
    dashboard = get_performance_dashboard()
    return dashboard.monitor.get_performance_summary()


@router.get("/api/v1/performance/response-times")
async def get_response_time_metrics(minutes: int = 5):
    """
    Get response time metrics for the specified time window
    """
    dashboard = get_performance_dashboard()
    return dashboard.monitor.get_response_time_metrics(minutes)


@router.get("/api/v1/performance/endpoints")
async def get_endpoint_performance(minutes: int = 5):
    """
    Get performance metrics grouped by endpoint
    """
    dashboard = get_performance_dashboard()
    return dashboard.monitor.get_endpoint_metrics(minutes)


@router.get("/api/v1/performance/throughput")
async def get_throughput_metrics(minutes: int = 5):
    """
    Get throughput metrics for the specified time window
    """
    dashboard = get_performance_dashboard()
    return dashboard.monitor.get_throughput_metrics(minutes)