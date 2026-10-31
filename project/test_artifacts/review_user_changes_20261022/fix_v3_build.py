from pathlib import Path
p=Path('/home/ly/execution_continuity_review_20261022/v3/build.sh')
s=p.read_text()
s=s.replace('source /opt/ros/humble/setup.bash\nsource /home/ly/execution_continuity_review_20261022/v1/install/local_setup.bash\nsource /home/ly/execution_continuity_review_20261022/v2/install/local_setup.bash\n', "source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v2/run_full.sh)\nsource /home/ly/execution_continuity_review_20261022/v3/install/local_setup.bash\n")
p.write_text(s)
