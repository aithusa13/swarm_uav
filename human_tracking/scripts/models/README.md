# MobileNet-SSD Model Files

The `person_tracker.py` node loads two files from this directory:

```
MobileNetSSD_deploy.prototxt.txt
MobileNetSSD_deploy.caffemodel
```

These are **not committed** to the repository. Download them from the
original author's repo:

- https://github.com/chuanqi305/MobileNet-SSD

Place both files directly in this folder:

```
human_tracking/scripts/models/
├── MobileNetSSD_deploy.prototxt.txt
└── MobileNetSSD_deploy.caffemodel
```

## Quick Download

```bash
cd human_tracking/scripts/models

# Deploy prototxt
wget https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/voc/MobileNetSSD_deploy.prototxt \
     -O MobileNetSSD_deploy.prototxt.txt

# Caffemodel (check the chuanqi305/MobileNet-SSD README for the current
# download link, since it is hosted on Google Drive)
```

## License / Attribution

MobileNet-SSD is released by chuanqi305 under the MIT license. The 21-class
PASCAL VOC labels are used; the `person` class (index 15) is the only one
consumed by this node.
