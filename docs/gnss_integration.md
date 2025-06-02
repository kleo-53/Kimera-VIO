# Kimera-VIO with GNSS Integration

This fork extends [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO) by incorporating GNSS support into the stereo visual-inertial pipeline. This implementation adds loosely coupled GNSS integration via custom GTSAM factors. The GNSS data is treated as pose factors (position only).

Due to lack of access to raw GNSS measurements in the [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) dataset, groundtruth data with different noise levels were used instead.

Developed as part of a Bachelor's thesis at Saint Petersburg State University.

## Architecture

### Added Components
Main components:
* `GNSSVIODataProvider`: loads GNSS data from dataset
* `GnssStereoDataProviderModule`: synchronizes different measurements at one time
* `GnssStereoVisionImuFrontend`: passes GNSS to backend
* `GnssVioBackend`: performs graph optimization taking into account GNSS factors

And following:
* `GnssParams`: set for interaction with GNSS. The `BackendParams` were also modified
* `GnssFactor`: adds position constraint to pose in backend
* `ThreadSafeGnssBuffer`: threadsafe structure to manipulate GNSS data in the pipeline
* `Tests` to added modules

### Backend Integration

GNSS factors are injected using `BeforeOptimizeHook()` in the `RegularVioBackend`. No changes to iSAM2 optimizer needed. The algorithm is robust to noisy GNSS because it can switch loss function in GNSS factors to Huber/Tukey.

## Dataset Format

Simulated GNSS data should be located at `gnss0/data.csv`:

```csv
#timestamp;x;y;z
1403636585023555584;0.2;1.5;0.0
...
```

Transform from GNSS sensor to body frame is defined in `gnss0/sensor.yaml`.

## Build and Run

Inctruction in Windows OS:

1. Clone the [repository](https://github.com/kleo-53/Kimera-VIO), switch to the dev branch
2. In `kimera_vio_docker.bash` you need to change the volume to the required ones (with the dataset and with the code)
3. Open WSL, go to the folder with the project
4. Execute commands:
```bash
docker build -t kimera_vio -f scripts/docker/Dockerfile .
./scripts/docker/kimera_vio_docker.bash
```
5. In docker in the `Kimera-VIO/` folder, recreate the `build/` folder and build the library there:
```bash
rm -rf /root/Kimera-VIO/build/*
mkdir -p ~/Kimera-VIO/build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(($(nproc)-1))
```
6. Run the script: `../scripts/gnssVIOEuroc.bash -p /Euroc/MH_01_easy -log`

Use Jupyter notebooks in [evaluation repository](https://github.com/MIT-SPARK/Kimera-VIO-Evaluation) to visualize results and compare errors.

## Results

At optimal parameters, the implementation with GNSS behaves more accurately than without GNSSi. If you add noise to GNSS data or reduce their amount, then the accuracy of the constructed trajectory decreases, but is still comparable to the trajectory without GNSS.

## Links

* [Pull Request #258](https://github.com/MIT-SPARK/Kimera-VIO/pull/258)
* [Kimera-VIO Repository](https://github.com/MIT-SPARK/Kimera-VIO)

---

For research use. Feedback and collaboration welcome!
