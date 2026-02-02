#!/bin/bash
# Ferma tutti i servizi che usano le camere
# Usare prima della calibrazione stereo

echo "=== Fermando servizi camera ==="

# Stop mjpeg server
sudo systemctl stop clover-mjpeg.service 2>/dev/null && echo "[OK] clover-mjpeg fermato" || echo "[--] clover-mjpeg non attivo"

# Kill eventuali processi rimasti
pkill -f mjpeg_server 2>/dev/null
pkill -f cat_tracker 2>/dev/null
pkill -f cat_follower 2>/dev/null

# Riavvia nvargus-daemon per pulire lo stato
echo "Riavvio nvargus-daemon..."
sudo systemctl restart nvargus-daemon
sleep 3

# Verifica
echo ""
echo "=== Stato ==="
ps aux | grep -E "(mjpeg|cat_)" | grep -v grep || echo "[OK] Nessun processo camera attivo"

echo ""
echo "=== Camere disponibili ==="
timeout 5 gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=1 ! "video/x-raw(memory:NVMM),width=640,height=480" ! nvvidconv ! fakesink 2>&1 | grep -E "(Success|Error|Failed)" || echo "[OK] Test completato"

echo ""
echo "Pronto per calibrazione!"
