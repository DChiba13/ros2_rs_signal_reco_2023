作成者: 千葉大樹
作成日: 2023/10/31

概要
このパッケージはつくば市役所周辺の歩行者用の赤信号と青信号を認識し、"RedLight" と "GreenLight"をパブリッシュするパッケージです。
また。ラベリングの結果画像もパブリッシュします。

内側に黄色い人型がある歩行者用信号に限り使用可能です。

・使用方法
rs_signal_reco_sub.cppは他のプログラムからカメラの連続画像のmsgをパブリッシュし、そのmsgをこのソースコードでサブスクライブする必要があります
デフォルトで使用するならば、"/camera1/image" というトピック名で別のプログラムからパブリッシュしてください。
型は sensor_msgs::msg::Image型です。

rs_signal_reco_sub.cppは
歩行者用信号が赤信号であると認識した場合、 "ros2_rs_interfaces::msg::TrafficSignal型" で "RedLight" をIMAGE_THRESHフレーム毎にパブリッシュします。
歩行者用信号が赤信号であると認識した場合、 "ros2_rs_interfaces::msg::TrafficSignal型" で "GreenLight" をIMAGE_THRESHフレーム毎にパブリッシュします。
デフォルトで使用するならば、"light_msg" というトピック名で別のプログラムでサブスクライブしてください。

rs_signal_reco_sub.cppは
歩行者用信号の赤青判定結果画像を"sensor_msgs::msg::Image型"でパブリッシュします。
デフォルトで使用するならば、"signal_image"というトピック名で別のプログラムでサブスクライブしてください。

rs_signal_reco.lanch.pyの中に
 remappings=[
      ('camera1/image', 'signal_test')
    ]
という行がありますが、ここではトピック名のリマップを行っています。
必要無い場合はコメントアウトしてください。