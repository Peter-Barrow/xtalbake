from thortec import MTD1020T


with MTD1020T(port='/dev/ttyUSB0') as mtd:
    print(mtd)
