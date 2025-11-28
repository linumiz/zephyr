# Flash Controller Test – T2G

This sample demonstrates testing the standard flash controller and the SMIF flash controller on T2G devices.
Flash operations rely on SROM API calls executed on the M0 core. Since Zephyr’s current T2G M0 implementation does not yet provide the required system call infrastructure, a prebuilt hex file (**mtb-example-ce240757-hello-world.hex**) must be flashed to the M0 core.
This enables the M7 cores to perform flash erase/write operations.

---

## **Build Command**

```bash
rm -rf build/ && west build -p auto -b kit_t2g_c2d6m_lite/cyt4dnjbzs/m7_0 -s ./samples/drivers/flash_driver_test_t2g/
```

---

## **Flash M0 Core**

```bash
west flash \
  --hex-file /home/sanjay/mtw/Hello_World/build/APP_KIT_T2G_C-2D-6M_LITE/Debug/mtb-example-ce240757-hello-world.hex \
  --openocd /home/sanjay/Downloads/openocd-5.11.0.4042-linux/openocd/bin/openocd \
  --openocd-search /home/sanjay/Downloads/openocd-5.11.0.4042-linux/openocd/scripts/
```

---

## **Flash M7 Cores (Normal Flashing)**

```bash
west flash \
  --openocd /home/sanjay/Downloads/openocd-5.11.0.4042-linux/openocd/bin/openocd \
  --openocd-search /home/sanjay/Downloads/openocd-5.11.0.4042-linux/openocd/scripts/
```

---

## **Important Note**

The provided hex file will execute on the M7 core as well when flashed.
After flashing the M0 core with this hex file, **you must reflash the M7 core again using the normal M7 flash command** to run the actual test application.
