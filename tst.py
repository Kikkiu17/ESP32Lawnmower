numero_utente = int(input("Inserisci un numero: "))
scelta = ""
cont_p = 0
cont_n = 0
cont_nulli = 0
while True:
    # ciclo per controllare se la risposta che inserisce è valida
    while True:
        scelta = input("Vuoi inseririre un altro numero? rispondi con y o Y (si) oppure N o n (no): ").upper()
        if scelta == "N" or scelta == "Y":
            break # se è valida va avanti, va fino alla riga 15 (if scelta == "Y")
        else:
            print("Risposta non valida! Riprova") # altrimenti continua a chiedere

    if scelta == "Y":
        numero = int(input("Inserisci un altro numero: "))
   
        if numero > 0:
            cont_p = cont_p + 1
        elif numero == 0:
            cont_nulli = cont_nulli + 1
        else:
            cont_n = cont_n + 1
        # aggiorna il contatore con i nuovi numeri inseriti
    elif scelta == "N":
        if numero_utente > 0:
            cont_p = cont_p + 1
        elif numero_utente == 0:
            cont_nulli = cont_nulli + 1
        else:
            cont_n = cont_n + 1
        # aggiorna il contatore con il numero inserito all'inizio, però lo puoi fare anche prima del ciclo

        print("Hai inserito " + str(cont_p) + " numeri positivi, " + str(cont_n) + " numeri negativi, " + str(cont_nulli) + " numeri nulli")
        break
