# MobileNet-SSD Model Files

The `person_detector.py` node loads two files from this directory:

```
MobileNetSSD_deploy.prototxt.txt
MobileNetSSD_deploy.caffemodel
```

These are **not committed** to the repository. Download them from the
original author's repo:

- https://github.com/chuanqi305/MobileNet-SSD

Place both files directly in this folder:

```
area_scanning/scripts/models/
├── MobileNetSSD_deploy.prototxt.txt
└── MobileNetSSD_deploy.caffemodel
```

## Quick Download

```bash
cd area_scanning/scripts/models

wget https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/voc/MobileNetSSD_deploy.prototxt \
     -O MobileNetSSD_deploy.prototxt.txt
```

The caffemodel is hosted on Google Drive -- see the chuanqi305/MobileNet-SSD
README for the current download link.

## License / Attribution

MobileNet-SSD is released by chuanqi305 under the MIT license.
