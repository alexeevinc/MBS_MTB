## About The Project
The code creates a Multibody Model of a SRAM chain drive in Adams. The model is used to study chain oscilations and effects of thr derailleur parametrers on the feedback to the driver.

<img src="https://github.com/user-attachments/assets/c46653c6-52e8-474d-a440-ce55d7b0484a" alt="title_pic" width="500"/>


<img src="https://github.com/user-attachments/assets/6a39e70d-57ed-449e-91d7-1ef414bfd711" alt="title_pic" width="500"/>

The positions for each pin are calculated analytically using the methodology described in https://link.springer.com/article/10.1007/s11044-010-9207-x


## Requirements

The code uses a Adams-Python library of https://marketplace.visualstudio.com/items?itemName=savvyanalyst.msc-adams

Note: you do not need to install the library on yoour UDE to create the model as it is. Pass the code to Adams and it will use its own interpretor to utilize the commands.

 If nessecary, install the mossing python-packages in the Adams command promt using pip
   ```sh
    adams2023_3 python -m pip install matplotlib
   ```
## Usage

- Use the main to calculate an write the individual positions of the chain pins
- Use the main_2d to create the model in Adams


