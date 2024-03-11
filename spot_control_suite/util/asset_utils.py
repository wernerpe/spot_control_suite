from pydrake.all import PackageMap
import os

def get_asset_path():
    path_assets = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path_assets += '/assets'
    return path_assets

def AddSpotRemote(parser):
    parser.package_map().AddRemote(
        package_name="spot_description",
        params=PackageMap.RemoteParams(
            urls=[
                f"https://github.com/wrangel-bdai/spot_ros2/archive/20965ef7bba98598ee10878c7b54e6ef28a300c6.tar.gz"
            ],
            sha256=("20a4f12896b04cc73e186cf876bf2c7e905ee88f8add8ea51bf52dfc888674b4"),
            strip_prefix="spot_ros2-20965ef7bba98598ee10878c7b54e6ef28a300c6/spot_description/",
        ),
    )