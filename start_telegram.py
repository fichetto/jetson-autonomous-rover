#!/usr/bin/env python3
"""
CLOVER Telegram Bot Launcher

Starts the Telegram bot for remote rover control and monitoring.

Usage:
    python start_telegram.py
    python start_telegram.py --token YOUR_BOT_TOKEN
    python start_telegram.py --config config/telegram_config.yaml

Environment variables:
    TELEGRAM_BOT_TOKEN: Bot token from @BotFather
    TELEGRAM_AUTHORIZED_USERS: Comma-separated list of user IDs
"""

import os
import sys
import argparse
import signal
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).parent
sys.path.insert(0, str(PROJECT_ROOT))

from loguru import logger

# Configure logging
logger.remove()
logger.add(
    sys.stderr,
    format="<green>{time:YYYY-MM-DD HH:mm:ss}</green> | <level>{level: <8}</level> | <cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> - <level>{message}</level>",
    level="INFO"
)
logger.add(
    PROJECT_ROOT / "logs" / "telegram_bot.log",
    rotation="10 MB",
    retention="7 days",
    level="DEBUG"
)


def load_config_file(config_path: str) -> dict:
    """Load configuration from YAML file."""
    try:
        import yaml
        with open(config_path, 'r') as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        logger.warning(f"Config file not found: {config_path}")
        return {}
    except Exception as e:
        logger.error(f"Error loading config: {e}")
        return {}


def create_default_config():
    """Create default telegram config file if it doesn't exist."""
    config_path = PROJECT_ROOT / "config" / "telegram_config.yaml"

    if config_path.exists():
        return

    default_config = """# CLOVER Telegram Bot Configuration

# Bot token from @BotFather (REQUIRED)
# Get one by talking to @BotFather on Telegram
bot_token: ""

# Authorized user IDs (optional)
# If empty, anyone can control the rover (NOT RECOMMENDED for production)
# Get your user ID by sending /id to the bot
authorized_users: []
  # - 123456789
  # - 987654321

# Arduino connection settings
modbus:
  port: "/dev/ttyACM0"
  baudrate: 115200

# Status monitoring interval (seconds)
status_interval: 30

# Battery alert settings
battery:
  warning_voltage: 10.0
  critical_voltage: 9.0
"""

    try:
        config_path.parent.mkdir(parents=True, exist_ok=True)
        with open(config_path, 'w') as f:
            f.write(default_config)
        logger.info(f"Created default config at: {config_path}")
    except Exception as e:
        logger.error(f"Failed to create config: {e}")


def get_token_interactively() -> str:
    """Prompt user for bot token if not provided."""
    print("\n" + "=" * 60)
    print("CLOVER Telegram Bot Setup")
    print("=" * 60)
    print("\nPer ottenere un token per il bot Telegram:")
    print("1. Apri Telegram e cerca @BotFather")
    print("2. Invia /newbot")
    print("3. Segui le istruzioni per creare il bot")
    print("4. Copia il token fornito")
    print("\n" + "-" * 60)

    token = input("\nInserisci il token del bot: ").strip()

    if not token:
        print("\n❌ Token non fornito. Uscita.")
        sys.exit(1)

    # Save to config file for future use
    try:
        import yaml
        config_path = PROJECT_ROOT / "config" / "telegram_config.yaml"
        config = {}
        if config_path.exists():
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f) or {}

        config['bot_token'] = token

        with open(config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)

        print(f"\n✅ Token salvato in: {config_path}")
    except Exception as e:
        logger.warning(f"Could not save token to config: {e}")

    return token


def main():
    parser = argparse.ArgumentParser(
        description="CLOVER Rover Telegram Bot",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                           # Use config file or environment
  %(prog)s --token YOUR_TOKEN        # Specify token directly
  %(prog)s --users 123,456           # Restrict to specific users

Environment Variables:
  TELEGRAM_BOT_TOKEN          Bot token from @BotFather
  TELEGRAM_AUTHORIZED_USERS   Comma-separated user IDs
        """
    )

    parser.add_argument(
        '--token', '-t',
        help='Telegram bot token'
    )
    parser.add_argument(
        '--users', '-u',
        help='Comma-separated list of authorized user IDs'
    )
    parser.add_argument(
        '--service-url', '-s',
        default='http://localhost:8081',
        help='RoverService URL (default: http://localhost:8081)'
    )
    parser.add_argument(
        '--config', '-c',
        default='config/telegram_config.yaml',
        help='Path to config file'
    )
    parser.add_argument(
        '--setup',
        action='store_true',
        help='Interactive setup mode'
    )

    args = parser.parse_args()

    # Create default config if needed
    create_default_config()

    # Load configuration (priority: args > env > config file)
    config_path = PROJECT_ROOT / args.config
    config = load_config_file(str(config_path))

    # Get bot token
    token = (
        args.token or
        os.environ.get('TELEGRAM_BOT_TOKEN') or
        config.get('bot_token')
    )

    # Interactive setup if no token
    if not token or args.setup:
        token = get_token_interactively()

    # Get authorized users
    authorized_users = None
    if args.users:
        authorized_users = [int(uid.strip()) for uid in args.users.split(',')]
    elif os.environ.get('TELEGRAM_AUTHORIZED_USERS'):
        authorized_users = [
            int(uid.strip())
            for uid in os.environ.get('TELEGRAM_AUTHORIZED_USERS').split(',')
        ]
    elif config.get('authorized_users'):
        authorized_users = config.get('authorized_users')

    # Get service URL
    service_url = args.service_url
    if config.get('service_url'):
        service_url = config['service_url']

    # Print startup info
    print("\n" + "=" * 60)
    print("🤖 CLOVER Telegram Bot")
    print("=" * 60)
    print(f"\n🌐 RoverService: {service_url}")
    if authorized_users:
        print(f"👥 Authorized users: {authorized_users}")
    else:
        print("⚠️  WARNING: No user restrictions - anyone can control the rover!")
    print(f"📁 Config: {config_path}")
    print(f"📝 Logs: {PROJECT_ROOT / 'logs' / 'telegram_bot.log'}")
    print("\n" + "-" * 60)
    print("Avvio bot in corso...")
    print("Premi Ctrl+C per terminare")
    print("-" * 60 + "\n")

    # Import and start bot
    try:
        from src.telegram import CloverTelegramBot

        bot = CloverTelegramBot(
            token=token,
            authorized_users=authorized_users,
            service_url=service_url
        )

        # Handle graceful shutdown
        def signal_handler(sig, frame):
            print("\n\n⏹️  Arresto bot in corso...")
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        # Run the bot
        bot.run()

    except ImportError as e:
        logger.error(f"Import error: {e}")
        print("\n❌ Dipendenze mancanti. Installa con:")
        print("   pip install python-telegram-bot")
        sys.exit(1)
    except Exception as e:
        logger.exception(f"Bot error: {e}")
        print(f"\n❌ Errore: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
