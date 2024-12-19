# EMUC-B202-W1

The can board embedded in the industrial pc used for the Greek tractor ([vbox-3611-poex](https://www.sintrones.com/VBOX-3611-POEX.html)) is EMUC-B202-W1, from Innodisk, and it is described in this [url](https://www.innodisk.com/en/products/embedded-peripheral/communication/emuc-b202).

In the previous url you can find the latests drivers for that board for Linux and Window operating systems. At the time of writing this document (2022-11-16) the driver (EMUC_B202_SocketCAN_driver) has version 3.6 and the release date was 2022-06-24.

The zip file downloaded from the previous url, containing the driver for the can board and other tools, is saved in the repo for this project, so we will always have at hand a driver that can allow us to use the can board embedded in the industrial pc, even in the case that one day the url mentioned before is down.

In the downloaded folder there is a how-to-install guide for the driver in Linux. I will provide a mechanism to install automatically the driver for the can board for Ubuntu in the installation scripts provided for this project. However, if you are in doubts, or you have any error when installing the driver, please refer to the guide.
