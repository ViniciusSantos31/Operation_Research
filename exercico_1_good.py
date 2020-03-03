import mySolver as ms
import sys

if (len(sys.argv) == 2):

    SOLVE = ms.mySolver() ## Construtor da classe

    SOLVE.readtxt(sys.argv[1]) ## leitor de instância

    SOLVE.primal_solution() ## Calcula o Primal

    SOLVE.dual_solution() ## Calcula o Dual a partir do Primal

    SOLVE.get_atributes("PrimaL") ## Retirna os valores das variaveis(variaveis, folgas/excessos, folgas complementares/excessos complementares) e da solucao do problema
    SOLVE.get_atributes("dUAl")

elif len(sys.argv) > 2:
    print("many arguments!")
    print("\ttry:\n            name-of-arquive.py name-instance.txt")
else:
    print("instance not found!")
    print("\ttry:\n            name-of-arquive.py name-instance.txt")


