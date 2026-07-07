$ProjectRoot = Split-Path -Parent $PSScriptRoot
$Connection = if ($env:UAS_IMAGE_RX_CONNECTION) { $env:UAS_IMAGE_RX_CONNECTION } else { "udpin:0.0.0.0:14551" }
$ExtraArgs = @()
if ($args.Count -gt 0) {
    $ExtraArgs = $args
}

Set-Location $ProjectRoot
python src\image_stream_receiver.py --connection $Connection @ExtraArgs
