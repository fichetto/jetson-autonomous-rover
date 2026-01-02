"""
CLOVER Telegram Bot Module

Provides remote control and monitoring of the rover via Telegram.
"""

from .telegram_bot import CloverTelegramBot
from .rover_interface import RoverInterface

__all__ = ['CloverTelegramBot', 'RoverInterface']
