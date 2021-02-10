with open("CONFIG_REGISTERS.txt", "r", encoding = "utf-8") as fil:
    for rad in fil:
        radlista = rad.split()                # Ett trebokstavsord per rad
        adress = radlista[0].strip()
        register = radlista[1].strip()
        beskrivning = " ".join(radlista[2:len(radlista)-2]).strip()
        sträng = "#define "+register+""
        for i in range(15-len(register)):
            sträng += " "

        sträng += adress
        sträng += "        //"
        sträng += beskrivning
        print(sträng)
