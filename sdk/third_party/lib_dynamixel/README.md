# lib_dynamixelの取り込み元

[SHINOBI-organization/lib_dynamixel](https://github.com/SHINOBI-organization/lib_dynamixel)のコミット
`6101b8b3fd7383e4661219b8fbb732a9a1cb9faa`から、SDKが使用する`src/`と`download/`を取り込み、以下を変更しました。
submoduleやビルド時のパッチ適用は不要です。

- `src/dynamixel_communicator_main.cpp`：通常SyncReadでProtocol 2.0のLength・CRC・byte stuffingを検証し、IDと応答数を確認。Return Delayを含めて応答待ち時間を計算。
  通常SyncReadの抽出項目上限を10から16へ拡張。SDKは既存の64〜146番地の同一応答から、位置・速度の内部軌道も抽出します。
- `src/protocol2_status.h`：上記のステータスパケット解析処理を追加。
- `download/port_handler_mac.{h,cpp}`：共通の`PortHandler` APIに接続するmacOSバックエンドを追加。
  `IOSSIOSPEED`で1 Mbpsを指定し、応答タイマーには単調時計を使用します。
  `latency_timer`は応答待ちの計算値であり、macOSのFTDIドライバ設定は変更しません。

`download/port_handler*`はROBOTISのDynamixel SDK由来です。各ファイルのCopyright 2017 ROBOTIS CO., LTD.・著者名・Apache-2.0表記を保持し、ライセンス本文を`LICENSE-ROBOTIS`に同梱しています。
