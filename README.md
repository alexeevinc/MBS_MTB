## About The Project
The code creates a Multibody Model of a SRAM chain drive in Adams. The model is used to study chain oscilations and effects of thr derailleur parametrers on the feedback to the driver.

<img width="1010" height="789" alt="Image" src="https://github.com/user-attachments/assets/16932611-291f-4b10-9d10-0bd9b82aa1e0" />

<img width="1150" height="784" alt="Image" src="https://github.com/user-attachments/assets/58787288-6709-4c87-a9aa-d16ca1e6250a" />

<img width="1429" height="349" alt="Image" src="https://github.com/user-attachments/assets/938341bc-3292-47ed-ad80-a6a674c6d78f" />

<img width="948" height="651" alt="Image" src="https://github.com/user-attachments/assets/33c0ecb2-2d40-4dcf-b212-1f04c80c7f0b" />



The positions for each pin are calculated analytically using the methodology described in https://link.springer.com/article/10.1007/s11044-010-9207-x


## Requirements

The code uses a Adams-Python library of https://marketplace.visualstudio.com/items?itemName=savvyanalyst.msc-adams

Note: you do not need to install the library on yoour UDE to create the model as it is. Pass the code to Adams and it will use its own interpretor to utilize the commands.

 If nessecary, install the missing python-packages in the Adams command promt using pip
   ```sh
    adams2023_3 python -m pip install matplotlib
   ```
## Usage

- Use the `main.py` to calculate an write the individual positions of the chain pins
- Use the `main_2d.py` to create the model in Adams


