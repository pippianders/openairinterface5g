<table style="border-collapse: collapse; border: none;">
  <tr style="border-collapse: collapse; border: none;">
    <td style="border-collapse: collapse; border: none;">
      <a href="http://www.openairinterface.org/">
         <img src="./images/oai_final_logo.png" alt="" border=3 height=50 width=150>
         </img>
      </a>
    </td>
    <td style="border-collapse: collapse; border: none; vertical-align: center;">
      <b><font size = "5">OAI 7:2 Frontahul Interface 5G SA tutorial</font></b>
    </td>
  </tr>
</table>

**Table of Contents**

[[_TOC_]]

#  1. Prerequisites

## 1.1 DPDK(Data Plane Development Kit)

Download DPDK version 20.05.0
```bash
wget http://fast.dpdk.org/rel/dpdk-20.05.tar.xz
```

DPDK Compilation
```bash
tar -xvf dpdk-20.05.tar.xz
cd dpdk-20.05

meson build
cd build
sudo ninja
sudo ninja install

make install T=x86_64-native-linuxapp-gcc
```

## 1.2 Setup 

Refer following link for setup configuration

https://docs.o-ran-sc.org/projects/o-ran-sc-o-du-phy/en/latest/Setup-Configuration_fh.html

### 1.2.1 RHEL

Update Linux boot arguments
```bash
BOOT_IMAGE=(hd0,gpt2)/vmlinuz-4.18.0-425.10.1.rt7.220.el8_7.x86_64 root=/dev/mapper/rhel_skylark-root ro crashkernel=auto resume=/dev/mapper/rhel_skylark-swap rd.lvm.lv=rhel_skylark/root rd.lvm.lv=rhel_skylark/swap rhgb quiet igb.max_vfs=2 intel_iommu=on iommu=pt intel_pstate=disable nosoftlockup tsc=nowatchdog mitigations=off cgroup_memory=1 cgroup_enable=memory mce=off idle=poll hugepagesz=1G hugepages=40 hugepagesz=2M hugepages=0 default_hugepagesz=1G selinux=0 enforcing=0 nmi_watchdog=0 softlockup_panic=0 audit=0 skew_tick=1 skew_tick=1 isolcpus=managed_irq,domain,0-2,8-17 intel_pstate=disable nosoftlockup tsc=reliable
```
### 1.2.1 Ubuntu

Install real timer kernel followed by updating boot arguments
```bash
isolcpus=0-2,8-17 nohz=on nohz_full=0-2,8-17 rcu_nocbs=0-2,8-17 rcu_nocb_poll nosoftlockup default_hugepagesz=1GB hugepagesz=1G hugepages=10 amd_iommu=on iommu=pt
```
Use isolated CPU 0-2 for DPDK/ORAN, CPU 8 for ru_thread in our example config

## 1.3 PTP configuration

Refer following link for PTP configuration

https://docs.o-ran-sc.org/projects/o-ran-sc-o-du-phy/en/latest/PTP-configuration_fh.html

# 2. Build OAI-FHI gNB

## 2.1 Build ORAN Fronthaul Interface Library

Download ORAN FHI library
```bash
git clone https://gerrit.o-ran-sc.org/r/o-du/phy.git
cd phy
git checkout oran_release_bronze_v1.1
```

Apply patches (available in oai_folder/cmake_targets/tools/oran_fhi_integration_patches/)
```bash
git apply oran-fhi-1-compile-libxran-using-gcc-and-disable-avx512.patch
git apply oran-fhi-2-return-correct-slot_id.patch
git apply oran-fhi-3-disable-pkt-validate-at-process_mbuf.patch
git apply oran-fhi-4-process_all_rx_ring.patch
git apply oran-fhi-5-remove-not-used-dependencies.patch
```

Set up the environment (change the path if you use different folders)

```bash
export XRAN_LIB_DIR=~/phy/fhi_lib/lib/build
export XRAN_DIR=~/phy/fhi_lib
export RTE_SDK=~/dpdk-20.05
export RTE_TARGET=x86_64-native-linuxapp-gcc
export RTE_INCLUDE=${RTE_SDK}/${RTE_TARGET}/include
```

Compile Fronthaul Interface Library
```bash
cd phy/fhi_lib/
./build.sh
```

## 2.2 Build OAI gNB

```bash
git clone https://gitlab.eurecom.fr/oai/openairinterface5g.git
cd openairinterface5g
git checkout develop
source oaienv
cd cmake_targets
./build_oai --gNB --ninja -t oran_fhlib_5g (Add, -I if it is build first time on server for installing external dependencies)
```

# 3. Configure Server and OAI gNB

## 3.1 Update following in fronthaul interface configuration - oran.fhi.json

```bash
 * DU, RU MAC Address
 * PCI address
```

## 3.2 Copy Fronthaul Configuration File

   ```
   cd ran_build/build
   cp ../../../targets/PROJECTS/GENERIC-NR-5GC/CONF/oran.conf.json .
   ```

## 3.2 Bind Devices

```bash
sudo modprobe vfio_pci
sudo /usr/local/bin/dpdk-devbind.py --bind vfio-pci 51:0e.0
sudo /usr/local/bin/dpdk-devbind.py --bind vfio-pci 51:0e.1
```

# 4. Run OAI gNB

## 4.1 Run OAI gNB

```bash
cd ran_build/build
cp ../../tools/oran_fhi_integration_patches/conf.json .

sudo ./nr-softmodem -O ../../../targets/PROJECTS/GENERIC-NR-5GC/CONF/oran.fh.band78.fr1.273PRB.conf --sa --reorder-thread-disable
```
