"""
GUI module for web interface.

Provides Flask routes, SocketIO handlers, and web application setup.
"""

from .web_app import create_app, socketio
from .socketio_handlers import emit_status

__all__ = ['create_app', 'socketio', 'emit_status']
