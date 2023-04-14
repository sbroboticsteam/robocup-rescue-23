# Setting up networking for virtual environments
A quick guide to basic networking using VMware/VirtualBox on Windows.

## VMware:
1. Open VMware and select your Ubuntu (or other) virtual machine.
2. Click on **Edit virtual machine settings**.
3. On the left-hand menu, click on **Network Adapter**.
4. In the network connection settings, select **Bridged: Connected directly to the physical network** and enable the **Replicate physical connection state** checkbox.
5. Click **OK**.

## VirtualBox:
1. Open VirtualBox and select your Ubuntu (or other) virtual machine.
2. Click on **Settings** in the top toolbar.
3. On the left-hand menu, select **Network**.
4. In the **Adapter 1** menu, click on **Attached to:** and select **Bridged Adapter** from the dropdown menu.
5. Now the **Name** dropdown should be enabled; click on it and select the appropriate network interface to connect to. (For example, if you're using Wi-Fi, select your Wi-Fi interface).
6. Click **OK**. 
