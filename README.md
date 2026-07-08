# Sci-Sky-Fc
Flight Controller inspired from MadFlight.
The limitation of Madflight is the available MCUs are quite limited, only Stm32f4, rp2040, rp2035, esp32s3, esp32. These MCU are either expensive or big. Madflight has alot of features that are advance while for hobby, they are not necessary. I want to create another FC by inspiring Madflight but only has enough feature so that can be run on cheaper and smaller MCU. But, I want also re-define what is really be, the modulation-what Madflight aims to be.
The design is based on highly-modular OOP and Hardware-Software Co-Design principle. It can be easily extended to more and more components without much effort, only inheritance. 
This is UML design, i.e., static design.
<img width="9252" height="5390" alt="flightcontroller class diagram" src="https://github.com/user-attachments/assets/9f536c75-401d-4090-a3d7-66743c7178a1" />
This is dynamic behavior, based on AADL notation.
<img width="1244" height="444" alt="design_4" src="https://github.com/user-attachments/assets/7c2af746-419a-4ac6-913f-aa2d1feec2d2" />
