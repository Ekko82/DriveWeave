# Third-party software and assets

The root MIT license covers original project source and documentation. It does
not relicense external simulators, their maps, vehicle meshes, or binary assets.
Download those distributions from their upstream projects. No CARLA or SUMO
runtime, original map file, or vehicle mesh is included in the source allowlist.

| Component | Use | Upstream license / source |
| --- | --- | --- |
| CARLA 0.9.16 | Native world, Tesla Model 3 blueprint, sensors and physics; official SUMO bridge imported from the user's installation | [CARLA license and asset terms](https://github.com/carla-simulator/carla/blob/0.9.16/LICENSE), [CARLA release](https://github.com/carla-simulator/carla/releases/tag/0.9.16) |
| Eclipse SUMO 1.28.0 | Traffic simulation, TraCI and native SUMO-GUI replay viewport | [EPL 2.0 / upstream licensing](https://eclipse.dev/sumo/docs/Libraries_Licensing.html) |
| NumPy | Numeric processing of actual sensor returns and poses | [NumPy license](https://github.com/numpy/numpy/blob/main/LICENSE.txt) |
| Pillow | Native image decoding, image processing and output | [Pillow license](https://github.com/python-pillow/Pillow/blob/main/LICENSE) |
| imageio-ffmpeg / FFmpeg | H.264 video encoding and decode verification | [imageio-ffmpeg license](https://github.com/imageio/imageio-ffmpeg/blob/main/LICENSE), [FFmpeg license and configuration](https://ffmpeg.org/legal.html) |
| lxml | XML support for the official SUMO integration | [lxml licensing](https://lxml.de/FAQ.html#what-is-the-license) |
| YOLOX-S | Official pretrained COCO ONNX RGB detector; preprocessing/decoding adapter based on the upstream protocol | [YOLOX Apache-2.0 license](https://github.com/Megvii-BaseDetection/YOLOX/blob/main/LICENSE), [official ONNX models](https://github.com/Megvii-BaseDetection/YOLOX/tree/main/demo/ONNXRuntime); local license copy: docs/licenses/YOLOX-APACHE-2.0.txt |
| ONNX Runtime | CPU inference of the downloaded detector | [MIT license](https://github.com/microsoft/onnxruntime/blob/main/LICENSE) |
| OpenCV | Detector input resizing and color conversion | [Apache-2.0 license](https://github.com/opencv/opencv/blob/4.x/LICENSE) |
| SciPy | Geometric clustering and short-term association assignment | [BSD license](https://github.com/scipy/scipy/blob/main/LICENSE.txt) |

CARLA and Tesla names identify simulator resources; this project has no brand
affiliation or endorsement. Historical semantic/instance masks come from simulator
ground truth. v3.1 separately uses actual pretrained YOLOX-S inference; weights
are downloaded from upstream and are not redistributed in this repository.

The historical SUMO-to-Blender prototype used **Car Concept — Eric Chadwick /
Darmstadt Graphics Group GmbH (2024), prepared for Khronos glTF Sample Assets,
based on Unity Fan's original public-domain model**, under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).
[Original source](https://github.com/KhronosGroup/glTF-Sample-Assets/tree/main/Models/CarConcept).
Its local axis, wheel pose, metric scale, contact height and materials were
modified; branded texture channels were removed. Background models were CC0
Innerscene assets. Those meshes and historical renders are not in the public
source allowlist. If distributing historical prototype media separately, keep
its original `assets/ASSET_LICENSES.txt` and attribution with the media. The
current native CARLA Tesla videos do not use the CarConcept mesh.
