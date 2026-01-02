"""
CLOVER Telegram Bot - Remote control and monitoring for the rover.

Features:
- Real-time status monitoring
- Mode switching (manual/autonomous/disabled)
- Emergency stop
- Basic movement commands
- Battery monitoring with alerts
"""

import asyncio
from typing import Optional, List, Set
from telegram import Update, InlineKeyboardButton, InlineKeyboardMarkup, BotCommand
from telegram.ext import (
    Application,
    CommandHandler,
    CallbackQueryHandler,
    ContextTypes,
    MessageHandler,
    filters
)
from telegram.constants import ParseMode
from loguru import logger

from .rover_interface import RoverInterface, RoverMode, RoverStatus


class CloverTelegramBot:
    """
    Telegram bot for CLOVER rover control.

    Provides commands for:
    - Status monitoring
    - Mode switching
    - Motor control
    - Emergency stop
    """

    def __init__(
        self,
        token: str,
        authorized_users: Optional[List[int]] = None,
        service_url: str = "http://localhost:8081",
        status_interval: float = 30.0
    ):
        """
        Initialize Telegram bot.

        Args:
            token: Telegram bot token from @BotFather
            authorized_users: List of authorized Telegram user IDs (None = allow all)
            service_url: URL of the RoverService (default: http://localhost:8081)
            status_interval: Interval for periodic status updates (seconds)
        """
        self.token = token
        self.authorized_users: Set[int] = set(authorized_users) if authorized_users else set()
        self.status_interval = status_interval

        # Initialize rover interface (HTTP client to RoverService)
        self.rover = RoverInterface(service_url=service_url)

        # Bot application
        self.app: Optional[Application] = None

        # Active chat IDs for broadcast messages
        self._active_chats: Set[int] = set()

        # Status monitoring task
        self._status_task: Optional[asyncio.Task] = None

        logger.info(f"CloverTelegramBot initialized")
        if self.authorized_users:
            logger.info(f"Authorized users: {self.authorized_users}")
        else:
            logger.warning("No user restrictions - anyone can control the rover!")

    def _is_authorized(self, user_id: int) -> bool:
        """Check if user is authorized to control the rover."""
        if not self.authorized_users:
            return True  # No restrictions
        return user_id in self.authorized_users

    async def _check_auth(self, update: Update) -> bool:
        """Check authorization and send error if not authorized."""
        user_id = update.effective_user.id
        if not self._is_authorized(user_id):
            await update.message.reply_text(
                "⛔ *Non autorizzato*\n\n"
                f"Il tuo User ID ({user_id}) non è nella lista degli utenti autorizzati.",
                parse_mode=ParseMode.MARKDOWN
            )
            logger.warning(f"Unauthorized access attempt from user {user_id}")
            return False
        return True

    # ==================== Command Handlers ====================

    async def cmd_start(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /start command - Show welcome message and main menu."""
        if not await self._check_auth(update):
            return

        # Register chat for broadcasts
        self._active_chats.add(update.effective_chat.id)

        keyboard = [
            [
                InlineKeyboardButton("📊 Stato", callback_data="status"),
                InlineKeyboardButton("🔌 Connetti", callback_data="connect")
            ],
            [
                InlineKeyboardButton("🎮 Manuale", callback_data="mode_manual"),
                InlineKeyboardButton("🤖 Autonomo", callback_data="mode_auto")
            ],
            [
                InlineKeyboardButton("⏹️ Stop Motori", callback_data="stop"),
                InlineKeyboardButton("🚨 E-STOP", callback_data="estop")
            ],
            [
                InlineKeyboardButton("🕹️ Controlli", callback_data="controls")
            ]
        ]
        reply_markup = InlineKeyboardMarkup(keyboard)

        await update.message.reply_text(
            "🤖 *CLOVER Rover Control*\n\n"
            "Benvenuto nel pannello di controllo del rover!\n\n"
            "Usa i pulsanti qui sotto o i comandi:\n"
            "• `/status` - Stato del rover\n"
            "• `/connect` - Connetti al rover\n"
            "• `/disconnect` - Disconnetti\n"
            "• `/manual` - Modalità manuale\n"
            "• `/auto` - Modalità autonoma\n"
            "• `/stop` - Ferma motori\n"
            "• `/estop` - Emergency stop\n"
            "• `/help` - Aiuto comandi",
            parse_mode=ParseMode.MARKDOWN,
            reply_markup=reply_markup
        )

    async def cmd_help(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /help command - Show available commands."""
        if not await self._check_auth(update):
            return

        help_text = """
🤖 *CLOVER Rover - Comandi Disponibili*

*Stato e Connessione:*
/status - Mostra stato attuale del rover
/connect - Connetti al rover via Modbus
/disconnect - Disconnetti dal rover

*Modalità Operative:*
/manual - Passa a modalità manuale
/auto - Passa a modalità autonoma
/disable - Disabilita il rover

*Controllo Motori:*
/stop - Ferma tutti i motori (soft stop)
/estop - Emergency stop (blocco immediato)
/release - Rilascia emergency stop

*Movimento (solo in modalità manuale):*
/forward [speed] - Avanti (default 0.3 m/s)
/backward [speed] - Indietro
/left [speed] - Strafe sinistra
/right [speed] - Strafe destra
/rotleft [speed] - Ruota antiorario (rad/s)
/rotright [speed] - Ruota orario

*Altro:*
/menu - Mostra menu principale
/id - Mostra il tuo User ID

*Note:*
• La velocità lineare è in m/s (max 1.0)
• La velocità angolare è in rad/s (max 2.5)
• Emergency stop richiede /release per sbloccare
        """
        await update.message.reply_text(help_text, parse_mode=ParseMode.MARKDOWN)

    async def cmd_status(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /status command - Show rover status."""
        if not await self._check_auth(update):
            return

        status = await self.rover.get_status()
        await update.message.reply_text(
            status.to_message(),
            parse_mode=ParseMode.MARKDOWN
        )

    async def cmd_connect(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /connect command - Connect to rover."""
        if not await self._check_auth(update):
            return

        msg = await update.message.reply_text("🔄 Connessione al rover in corso...")

        if await self.rover.connect():
            await msg.edit_text(
                "✅ *Connesso al rover!*\n\n"
                "Usa /status per vedere lo stato attuale.",
                parse_mode=ParseMode.MARKDOWN
            )
        else:
            await msg.edit_text(
                "❌ *Connessione fallita*\n\n"
                "Verifica che Arduino sia connesso via USB.",
                parse_mode=ParseMode.MARKDOWN
            )

    async def cmd_disconnect(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /disconnect command - Disconnect from rover."""
        if not await self._check_auth(update):
            return

        await self.rover.disconnect()
        await update.message.reply_text(
            "🔌 *Disconnesso dal rover*",
            parse_mode=ParseMode.MARKDOWN
        )

    async def cmd_manual(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /manual command - Switch to manual mode."""
        if not await self._check_auth(update):
            return

        if await self.rover.set_mode(RoverMode.MANUAL):
            await update.message.reply_text(
                "🎮 *Modalità MANUALE attivata*\n\n"
                "Puoi ora controllare il rover con i comandi di movimento.",
                parse_mode=ParseMode.MARKDOWN
            )
        else:
            await update.message.reply_text(
                "❌ *Errore nel cambio modalità*\n\n"
                "Verifica la connessione con /status",
                parse_mode=ParseMode.MARKDOWN
            )

    async def cmd_auto(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /auto command - Switch to autonomous mode."""
        if not await self._check_auth(update):
            return

        if await self.rover.set_mode(RoverMode.AUTONOMOUS):
            await update.message.reply_text(
                "🤖 *Modalità AUTONOMA attivata*\n\n"
                "Il rover seguirà la navigazione autonoma.",
                parse_mode=ParseMode.MARKDOWN
            )
        else:
            await update.message.reply_text(
                "❌ *Errore nel cambio modalità*",
                parse_mode=ParseMode.MARKDOWN
            )

    async def cmd_disable(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /disable command - Disable rover."""
        if not await self._check_auth(update):
            return

        await self.rover.stop_motors()
        await self.rover.set_mode(RoverMode.DISABLED)
        await update.message.reply_text(
            "⏹️ *Rover DISABILITATO*\n\n"
            "I motori sono stati fermati.",
            parse_mode=ParseMode.MARKDOWN
        )

    async def cmd_stop(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /stop command - Stop all motors."""
        if not await self._check_auth(update):
            return

        if await self.rover.stop_motors():
            await update.message.reply_text(
                "⏹️ *Motori fermati*",
                parse_mode=ParseMode.MARKDOWN
            )
        else:
            await update.message.reply_text(
                "❌ *Errore nell'arresto motori*",
                parse_mode=ParseMode.MARKDOWN
            )

    async def cmd_estop(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /estop command - Emergency stop."""
        if not await self._check_auth(update):
            return

        if await self.rover.emergency_stop():
            await update.message.reply_text(
                "🚨 *EMERGENCY STOP ATTIVATO*\n\n"
                "Tutti i motori sono bloccati.\n"
                "Usa /release per sbloccare.",
                parse_mode=ParseMode.MARKDOWN
            )
        else:
            await update.message.reply_text(
                "❌ *Errore nell'emergency stop*",
                parse_mode=ParseMode.MARKDOWN
            )

    async def cmd_release(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /release command - Release emergency stop."""
        if not await self._check_auth(update):
            return

        if await self.rover.release_emergency_stop():
            await update.message.reply_text(
                "✅ *Emergency stop rilasciato*\n\n"
                "Il rover può ora essere controllato.",
                parse_mode=ParseMode.MARKDOWN
            )
        else:
            await update.message.reply_text(
                "❌ *Errore nel rilascio e-stop*",
                parse_mode=ParseMode.MARKDOWN
            )

    async def cmd_forward(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /forward command - Move forward."""
        if not await self._check_auth(update):
            return

        speed = 0.3
        if context.args:
            try:
                speed = float(context.args[0])
                speed = max(0.1, min(1.0, speed))
            except ValueError:
                pass

        if await self.rover.move_forward(speed):
            await update.message.reply_text(f"⬆️ Avanti a {speed:.1f} m/s")
        else:
            await update.message.reply_text("❌ Errore - verifica modalità e connessione")

    async def cmd_backward(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /backward command - Move backward."""
        if not await self._check_auth(update):
            return

        speed = 0.3
        if context.args:
            try:
                speed = float(context.args[0])
                speed = max(0.1, min(1.0, speed))
            except ValueError:
                pass

        if await self.rover.move_backward(speed):
            await update.message.reply_text(f"⬇️ Indietro a {speed:.1f} m/s")
        else:
            await update.message.reply_text("❌ Errore - verifica modalità e connessione")

    async def cmd_left(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /left command - Strafe left."""
        if not await self._check_auth(update):
            return

        speed = 0.3
        if context.args:
            try:
                speed = float(context.args[0])
                speed = max(0.1, min(1.0, speed))
            except ValueError:
                pass

        if await self.rover.strafe_left(speed):
            await update.message.reply_text(f"⬅️ Sinistra a {speed:.1f} m/s")
        else:
            await update.message.reply_text("❌ Errore - verifica modalità e connessione")

    async def cmd_right(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /right command - Strafe right."""
        if not await self._check_auth(update):
            return

        speed = 0.3
        if context.args:
            try:
                speed = float(context.args[0])
                speed = max(0.1, min(1.0, speed))
            except ValueError:
                pass

        if await self.rover.strafe_right(speed):
            await update.message.reply_text(f"➡️ Destra a {speed:.1f} m/s")
        else:
            await update.message.reply_text("❌ Errore - verifica modalità e connessione")

    async def cmd_rotleft(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /rotleft command - Rotate counter-clockwise."""
        if not await self._check_auth(update):
            return

        speed = 1.0
        if context.args:
            try:
                speed = float(context.args[0])
                speed = max(0.1, min(2.5, speed))
            except ValueError:
                pass

        if await self.rover.rotate_left(speed):
            await update.message.reply_text(f"↺ Rotazione sinistra a {speed:.1f} rad/s")
        else:
            await update.message.reply_text("❌ Errore - verifica modalità e connessione")

    async def cmd_rotright(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /rotright command - Rotate clockwise."""
        if not await self._check_auth(update):
            return

        speed = 1.0
        if context.args:
            try:
                speed = float(context.args[0])
                speed = max(0.1, min(2.5, speed))
            except ValueError:
                pass

        if await self.rover.rotate_right(speed):
            await update.message.reply_text(f"↻ Rotazione destra a {speed:.1f} rad/s")
        else:
            await update.message.reply_text("❌ Errore - verifica modalità e connessione")

    async def cmd_id(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /id command - Show user's Telegram ID."""
        user = update.effective_user
        await update.message.reply_text(
            f"👤 *Il tuo User ID:* `{user.id}`\n"
            f"📛 *Username:* @{user.username or 'N/A'}\n\n"
            "Usa questo ID per la configurazione degli utenti autorizzati.",
            parse_mode=ParseMode.MARKDOWN
        )

    async def cmd_menu(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /menu command - Show main menu."""
        await self.cmd_start(update, context)

    # ==================== Callback Query Handlers ====================

    async def callback_handler(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle inline keyboard button presses."""
        query = update.callback_query
        await query.answer()

        user_id = update.effective_user.id
        if not self._is_authorized(user_id):
            await query.edit_message_text("⛔ Non autorizzato")
            return

        action = query.data

        if action == "status":
            status = await self.rover.get_status()
            await query.edit_message_text(
                status.to_message(),
                parse_mode=ParseMode.MARKDOWN
            )

        elif action == "connect":
            await query.edit_message_text("🔄 Connessione in corso...")
            if await self.rover.connect():
                status = await self.rover.get_status()
                await query.edit_message_text(
                    "✅ *Connesso!*\n\n" + status.to_message(),
                    parse_mode=ParseMode.MARKDOWN
                )
            else:
                await query.edit_message_text(
                    "❌ *Connessione fallita*\n\nVerifica Arduino.",
                    parse_mode=ParseMode.MARKDOWN
                )

        elif action == "mode_manual":
            if await self.rover.set_mode(RoverMode.MANUAL):
                await query.edit_message_text(
                    "🎮 *Modalità MANUALE*\n\nControllo manuale attivo.",
                    parse_mode=ParseMode.MARKDOWN
                )
            else:
                await query.edit_message_text("❌ Errore cambio modalità")

        elif action == "mode_auto":
            if await self.rover.set_mode(RoverMode.AUTONOMOUS):
                await query.edit_message_text(
                    "🤖 *Modalità AUTONOMA*\n\nNavigazione autonoma attiva.",
                    parse_mode=ParseMode.MARKDOWN
                )
            else:
                await query.edit_message_text("❌ Errore cambio modalità")

        elif action == "stop":
            await self.rover.stop_motors()
            await query.edit_message_text(
                "⏹️ *Motori fermati*",
                parse_mode=ParseMode.MARKDOWN
            )

        elif action == "estop":
            await self.rover.emergency_stop()
            await query.edit_message_text(
                "🚨 *EMERGENCY STOP*\n\nUsa /release per sbloccare.",
                parse_mode=ParseMode.MARKDOWN
            )

        elif action == "controls":
            keyboard = [
                [InlineKeyboardButton("⬆️", callback_data="mv_forward")],
                [
                    InlineKeyboardButton("⬅️", callback_data="mv_left"),
                    InlineKeyboardButton("⏹️", callback_data="mv_stop"),
                    InlineKeyboardButton("➡️", callback_data="mv_right")
                ],
                [InlineKeyboardButton("⬇️", callback_data="mv_backward")],
                [
                    InlineKeyboardButton("↺", callback_data="mv_rotleft"),
                    InlineKeyboardButton("↻", callback_data="mv_rotright")
                ],
                [InlineKeyboardButton("🔙 Menu", callback_data="back_menu")]
            ]
            await query.edit_message_text(
                "🕹️ *Controlli Movimento*\n\n"
                "Usa i pulsanti per muovere il rover.\n"
                "(Solo in modalità manuale)",
                parse_mode=ParseMode.MARKDOWN,
                reply_markup=InlineKeyboardMarkup(keyboard)
            )

        elif action == "mv_forward":
            await self.rover.move_forward(0.3)
            await query.answer("⬆️ Avanti")

        elif action == "mv_backward":
            await self.rover.move_backward(0.3)
            await query.answer("⬇️ Indietro")

        elif action == "mv_left":
            await self.rover.strafe_left(0.3)
            await query.answer("⬅️ Sinistra")

        elif action == "mv_right":
            await self.rover.strafe_right(0.3)
            await query.answer("➡️ Destra")

        elif action == "mv_rotleft":
            await self.rover.rotate_left(1.0)
            await query.answer("↺ Rotazione sinistra")

        elif action == "mv_rotright":
            await self.rover.rotate_right(1.0)
            await query.answer("↻ Rotazione destra")

        elif action == "mv_stop":
            await self.rover.stop_motors()
            await query.answer("⏹️ Stop")

        elif action == "back_menu":
            keyboard = [
                [
                    InlineKeyboardButton("📊 Stato", callback_data="status"),
                    InlineKeyboardButton("🔌 Connetti", callback_data="connect")
                ],
                [
                    InlineKeyboardButton("🎮 Manuale", callback_data="mode_manual"),
                    InlineKeyboardButton("🤖 Autonomo", callback_data="mode_auto")
                ],
                [
                    InlineKeyboardButton("⏹️ Stop Motori", callback_data="stop"),
                    InlineKeyboardButton("🚨 E-STOP", callback_data="estop")
                ],
                [
                    InlineKeyboardButton("🕹️ Controlli", callback_data="controls")
                ]
            ]
            await query.edit_message_text(
                "🤖 *CLOVER Rover Control*\n\n"
                "Seleziona un'opzione:",
                parse_mode=ParseMode.MARKDOWN,
                reply_markup=InlineKeyboardMarkup(keyboard)
            )

    # ==================== Background Tasks ====================

    async def _battery_monitor(self, app: Application):
        """Background task to monitor battery and send alerts."""
        while True:
            try:
                await asyncio.sleep(60)  # Check every minute

                warning = await self.rover.check_battery_warning()
                if warning and self._active_chats:
                    for chat_id in self._active_chats:
                        try:
                            await app.bot.send_message(
                                chat_id=chat_id,
                                text=warning,
                                parse_mode=ParseMode.MARKDOWN
                            )
                        except Exception as e:
                            logger.error(f"Failed to send battery warning to {chat_id}: {e}")
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Battery monitor error: {e}")

    # ==================== Bot Lifecycle ====================

    async def post_init(self, app: Application):
        """Called after bot initialization."""
        # Set bot commands
        commands = [
            BotCommand("start", "Avvia il bot e mostra menu"),
            BotCommand("status", "Mostra stato del rover"),
            BotCommand("connect", "Connetti al rover"),
            BotCommand("disconnect", "Disconnetti dal rover"),
            BotCommand("manual", "Modalità manuale"),
            BotCommand("auto", "Modalità autonoma"),
            BotCommand("stop", "Ferma motori"),
            BotCommand("estop", "Emergency stop"),
            BotCommand("release", "Rilascia e-stop"),
            BotCommand("menu", "Mostra menu"),
            BotCommand("help", "Mostra aiuto"),
        ]
        await app.bot.set_my_commands(commands)

        # Start battery monitor
        self._status_task = asyncio.create_task(self._battery_monitor(app))

        logger.info("Telegram bot initialized and ready")

    async def post_shutdown(self, app: Application):
        """Called during bot shutdown."""
        # Cancel background tasks
        if self._status_task:
            self._status_task.cancel()
            try:
                await self._status_task
            except asyncio.CancelledError:
                pass

        # Disconnect from rover
        await self.rover.disconnect()

        logger.info("Telegram bot shutdown complete")

    def run(self):
        """Start the bot (blocking)."""
        # Build application
        self.app = (
            Application.builder()
            .token(self.token)
            .post_init(self.post_init)
            .post_shutdown(self.post_shutdown)
            .build()
        )

        # Add handlers
        self.app.add_handler(CommandHandler("start", self.cmd_start))
        self.app.add_handler(CommandHandler("help", self.cmd_help))
        self.app.add_handler(CommandHandler("status", self.cmd_status))
        self.app.add_handler(CommandHandler("connect", self.cmd_connect))
        self.app.add_handler(CommandHandler("disconnect", self.cmd_disconnect))
        self.app.add_handler(CommandHandler("manual", self.cmd_manual))
        self.app.add_handler(CommandHandler("auto", self.cmd_auto))
        self.app.add_handler(CommandHandler("disable", self.cmd_disable))
        self.app.add_handler(CommandHandler("stop", self.cmd_stop))
        self.app.add_handler(CommandHandler("estop", self.cmd_estop))
        self.app.add_handler(CommandHandler("release", self.cmd_release))
        self.app.add_handler(CommandHandler("forward", self.cmd_forward))
        self.app.add_handler(CommandHandler("backward", self.cmd_backward))
        self.app.add_handler(CommandHandler("left", self.cmd_left))
        self.app.add_handler(CommandHandler("right", self.cmd_right))
        self.app.add_handler(CommandHandler("rotleft", self.cmd_rotleft))
        self.app.add_handler(CommandHandler("rotright", self.cmd_rotright))
        self.app.add_handler(CommandHandler("id", self.cmd_id))
        self.app.add_handler(CommandHandler("menu", self.cmd_menu))
        self.app.add_handler(CallbackQueryHandler(self.callback_handler))

        logger.info("Starting Telegram bot...")
        self.app.run_polling(allowed_updates=Update.ALL_TYPES)
