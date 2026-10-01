# ファイルがでかいので、追跡対象外としています。

Gazeboで使用するSTLメッシュファイルを格納するディレクトリです。

Gazeboでメッシュを使用するには、環境変数GZ_SIM_RESOURCE_PATHにこのディレクトリを追加してください。

```bash
export GZ_SIM_RESOURCE_PATH=$HOME/sirius_jazzy_ws/src/sirius/sirius_description/meshes:$GZ_SIM_RESOURCE_PATH
```