SET PATH=C:\Keil_v5\ARM\ARMCC\Bin;C:\TwinCAT\Common64;C:\TwinCAT\Common32;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Program Files (x86)\Windows Kits\8.1\Windows Performance Toolkit\;C:\Program Files\Microsoft SQL Server\110\Tools\Binn\;C:\Program Files\Git\cmd;C:\Program Files\MATLAB\R2016a\runtime\win64;C:\Program Files\MATLAB\R2016a\bin;C:\Program Files\MATLAB\R2016a\polyspace\bin;C:\Program Files (x86)\Altium Designer Winter 09\System
SET CPU_TYPE=STM32F407ZE
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=STM32F40_41xxx
SET CPU_CLOCK=0x00B71B00
"C:\Keil_v5\ARM\ARMCC\Bin\ArmAsm" --Via "stm32f40_41xxx\startup_stm32f40_41xxx._ia"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\system_stm32f4xx.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\misc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\stm32f4xx_adc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\stm32f4xx_dma.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\stm32f4xx_exti.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\stm32f4xx_fsmc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\stm32f4xx_gpio.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\stm32f4xx_rcc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\stm32f4xx_syscfg.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\stm32f4xx_tim.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\stm32f4xx_usart.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\stm32f4xx_spi.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\bsp.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\stm32f405hw.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\stm32f405appl.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\coeappl.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\ecatappl.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\aoeappl.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\ecataoe.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\foeappl.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\ecatcoe.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\bootmode.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\diag.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\ecatfoe.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\ecateoe.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\ecatslv.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\ecatsoe.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\emcy.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\eoeappl.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\mailbox.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\objdef.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "stm32f40_41xxx\sdoserv.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmLink" --Via "STM32F40_41xxx\STM32F40_41xxx.lnp"
"C:\Keil_v5\ARM\ARMCC\Bin\fromelf.exe" "STM32F40_41xxx\STM32F40_41xxx.axf" --i32combined --output "STM32F40_41xxx\STM32F40_41xxx.hex"
