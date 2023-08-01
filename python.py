'''
Scrivere un programma che chieda all'utente un intero maggiore di 0, rappresentante la capacità in kg di uno zaino.
Viene dunque mostrato un menù di scelta con le seguenti opzioni (effettuare per ogni opzione gli opportuni controlli):
Inserimento di un oggetto nello zaino, indicandone il peso.
Rimozione di un oggetto dallo zaino, indicandone la posizione.
Rimozione di più oggetti da uno zaino, indicando un peso e rimuovendo tutti gli oggetti con un peso almeno pari a quello indicato.
Visualizzazione dello stato dello zaino, indicando tutti gli oggetti con il loro peso, il peso attuale dello zaino e quanto manca al raggiungimento della capacità massima.
Visualizzazione di tutti gli oggetti che pesano più della media degli oggetti.
Non deve essere prevista uscita dal menù manuale, bensì deve avvenire automaticamente non appena si inserisce un oggetto che farebbe andare il peso dello zaino oltre la sua capacità (tale oggetto non va dunque inserito, si deve terminare direttamente).
'''

print("benvenuto al programma di gestione dello zaino!")

capacita = int(input("Inserisci la capacità dello zaino: "))
zaino = []

while True:
    print("1 - Inserimento di un oggetto nello zaino")
    print("2 - Rimozione di un oggetto nello zaino, indicandone la posizione")
    print("3 - Rimozione di più oggetti dallo zaino")
    print("4 - Visualizzazione dello stato dello zaino")
    print("5 - Visualizzazione degli oggetti che pesano più della media")
    print("6 - Uscita")
    
    scelta = int(input("Inserisci la scelta che vuoi fare: "))

    print()
    if scelta < 1 or scelta > 6:
        print("La scelta non è valida, riprova")
    else:
        if scelta == 1:
            peso_attuale = 0
            for i in range(len(zaino)):
                peso_attuale = peso_attuale + zaino[i]

            uscita = False
            peso = 0
            while True:
                peso = int(input("Inserisci il peso dell'oggetto che vuoi inserire: "))
                if peso < 0:
                    print("Il peso inserito non è valido! Riprova")
                elif peso + peso_attuale > capacita:
                    print("L'oggetto che stai provando ad inserire farebbe superare la capacità massima dello zaino")
                    uscita = True
                    break
                else:
                    break

            if uscita == True:
                break

            zaino.append(peso)
        elif scelta == 2:
            while True:
                posizione_oggetto = int(input("Inserisci la posizione dell'oggetto che vuoi rimuovere (da 1 a " + str(len(zaino)) + "): "))
                posizione_oggetto = posizione_oggetto - 1
                if posizione_oggetto < 0 or posizione_oggetto > len(zaino) - 1:
                    print("La posizione inserita non è valida")
                else:
                    break

            zaino.pop(posizione_oggetto)
        elif scelta == 3:
            peso = 0

            while True:
                peso = int(input("Inserisci il peso minimo: "))
                if peso < 0 or peso > max(zaino):
                    print("Il peso inserito non è valido")
                else:
                    break
            
            i = 0
            for oggetto in zaino:
                i += 1
                if oggetto >= peso:
                    print("Il valore maggiore di", peso, "è", oggetto, "alla posizione numero", i)
                    zaino.remove(oggetto) # qua ho usato remove per non fare casini che stavo provando a spiegare, usando for oggetto in zaino
                    
        elif scelta == 4:
            peso_attuale = 0
            for i in range(len(zaino)):
                peso_attuale = peso_attuale + zaino[i]

            rimanente = capacita - peso_attuale

            print("Il peso attuale dello zaino è", peso_attuale)
            print("La capacità rimanente è", rimanente)

            for i in range(len(zaino)):
                print("Il peso dell'oggetto numero", i + 1, "è", zaino[i])
        elif scelta == 5:
            media = sum(zaino) / len(zaino)

            for i in range(len(zaino)):
                if zaino[i] > media:
                    print("Il peso dell'oggetto numero", i + 1, "è", zaino[i])
        else:
            break

        print()