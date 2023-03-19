# jishupro
robot-programming code for tensegrity robot

## PCで実行
#### roscore
#### rosrun rosserial_python serial_node.py _port:=/dev/rfcomm9 _baud:=57600
rfcomm通信を用いてPCとesp32をbluetooth接続。rfcomm通信は無線(bluetooth)を有線(usb通信)みたいに扱える通信方式。
baudrateは何でもいいけど、esp32に書き込むやつと合わせる。
