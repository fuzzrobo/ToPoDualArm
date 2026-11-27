# 300 mmモデル

標準の16軸モデルは `robot.urdf` と `mujoco/robot.xml` です。パンチルトの差分は `pan_tilt.urdf` と `mujoco/pan_tilt.xml` にあり、サーボ2台、取り付け部品、正面RealSenseと光学TFを含みます。

パンチルト付きの18軸モデルを使うには、機体YAMLで次を指定します。

```yaml
model: 300mm
modules: [pan_tilt]
```

標準機に正面カメラを固定する場合は、実際の取付位置に合わせてTFを定義します。

SDKとROS 2は起動時に差分を読み込みます。SDKのPython APIからは `load_config(path).options.robot_urdf()` で組み立てたXMLを取得できます。

MuJoCoとPyYAMLが入った環境で、機体YAMLを指定します。Macでは `mjpython`、Linuxでは `python` を使います。`--config` を省略すると標準16軸で開きます。

```sh
mjpython models/300mm/mujoco/load_model.py --config config/hardware/follower.yaml
```
