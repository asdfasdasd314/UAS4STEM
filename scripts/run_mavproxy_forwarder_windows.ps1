param(
    [string]$ComPort = "COM5",
    [int]$BaudRate = 57600,
    [string]$MissionPlannerOut = "udp:127.0.0.1:14550",
    [string]$ImageReceiverOut = "udp:127.0.0.1:14551"
)

mavproxy.py --master=$ComPort --baudrate $BaudRate --out=$MissionPlannerOut --out=$ImageReceiverOut
