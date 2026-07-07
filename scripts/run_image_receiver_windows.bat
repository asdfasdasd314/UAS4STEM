@echo off
setlocal

set "PROJECT_ROOT=%~dp0.."
if "%UAS_IMAGE_RX_CONNECTION%"=="" (
  set "UAS_IMAGE_RX_CONNECTION=udpin:0.0.0.0:14551"
)

cd /d "%PROJECT_ROOT%"
python src\image_stream_receiver.py --connection %UAS_IMAGE_RX_CONNECTION% %*
