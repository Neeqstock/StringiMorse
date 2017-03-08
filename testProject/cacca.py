puzza = 6
print(3 * 'un' + 'ium')

if puzza == 5:
    print("no")
elif puzza == 7:
    print("neanche")
else:
    print("ecco!")

i = range(2, 22, 2)

print(list(i))

for n in i:
    print(n)
else:
    print("sgnacchere")

def pastaAlSugo(pappa):
    return pappa + "con molto sugo"

print(pastaAlSugo("Tom "))

def cantonian(frase: str, altrafrase: str = " ") -> None:
    if altrafrase == " ":
        print(frase)
    else:
        print(frase + " with fucili, end " + altrafrase)

print(cantonian("Men", "uniformi"))

def f(a, L=None):
    if L is None:
        L = []
    L.append(a)
    return L

print(f(5))
print(f(3))

def parrot(voltage, state=None, action='voom', type='Norwegian Blue'):
    print("-- This parrot wouldn't", action)
    print("if you put", voltage, "volts through it.")
    print("-- Lovely plumage, the", type)
    print("-- It's", state, "!")

parrot(voltage=220, type="brazilian", action="fly")

def scriviCacca(*strunzate):
    for strunzata in strunzate:
        print(strunzata + " e' una strunzata grossissima!")
    else:
        print("Basta strunzate")

scriviCacca("L'esame di Greco", "Essere venuto in bici", "Il Carrefour")
scriviCacca()

def cheeseshop(kind, *arguments, **keywords):
    print("-- Do you have any", kind, "?")
    print("-- I'm sorry, we're all out of", kind)
    for arg in arguments:
        print(arg)
    print("-" * 40)
    for kw in keywords:
        print(kw, ":", keywords[kw])

cheeseshop("Limburger", "It's very runny, sir.",
           "It's really very, VERY runny, sir.",
           shopkeeper="Michael Palin",
           client="John Cleese",
           sketch="Cheese Shop Sketch")

raggio = [2, 10]
for n in range(*raggio):
    print("cacca", n)

print(cantonian.__name__)

from collections import deque
queue = deque(["Eric", "John", "Michael"])
queue.append("Terry")           # Terry arrives
queue.append("Graham")          # Graham arrives
print(queue.popleft())               # The first to arrive now leaves
print(queue.popleft())         # The second to arrive now leaves
print(queue)                          # Remaining queue in order of arrival

listaRandom = ["a", "b", "c"]
altraLista = ["d", "e", "f"]
listaRandom.extend(altraLista)

print(listaRandom)

u = (("a", "b", "c"), (1, 2, 3, 4, 5))

print(u[1][2])