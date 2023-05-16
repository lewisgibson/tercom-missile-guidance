# tercom-missile-guidance

This repository showcases sophisticated C++ implementations of trajectory guidance calculations for cruise missiles, specifically focusing on topographic reconnaissance.

## TERCOM

TERCOM, short for **Ter**rain **Co**ntour **M**atching, is a method of navigation used in guided missile systems. It relies on matching the terrain features observed by an onboard sensor with a pre-existing digital map or database.

The utilization of TERCOM, distinguished for its exceptional precision stemming from the radar altimeter/contour map integration, offers substantial advantages over INS. By enabling the missile to navigate in closer proximity to obstacles and maintain lower altitudes, TERCOM enhances the stealth capability, rendering detection by ground radar considerably more challenging.

## Building

### Ubuntu

Requirements:

-   Bash
-   CMake >= 3.24.2
-   GCC >= 8

Use the existing build script:

```bash
./build.sh
```
