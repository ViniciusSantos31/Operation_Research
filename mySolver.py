from __future__ import print_function
from ortools.linear_solver import pywraplp
import numpy as np

class mySolver():
    
    def __init__(self):
        
        self.matrix = []
        self.dual_matrix = []
        
        self.var_p = []
        self.var_d = []
        
        self.res_p = []
        self.res_d = []
        
        self.obj_fun_p = []
        self.obj_fun_d = []
        
        self.resultado_p = None
        self.resultado_d = None 
        
        self.num_var_p = None
        self.num_var_d = None
        
        self.num_res_p = None
        self.num_res_d = None
        
        self.solucao_p = []
        self.solucao_d = []
        
        self.solver = pywraplp.Solver('ProgramacaoLinear',pywraplp.Solver.GLOP_LINEAR_PROGRAMMING)
        self.solver_d = pywraplp.Solver('ProgramacaoLinear',pywraplp.Solver.GLOP_LINEAR_PROGRAMMING)
        
        self.excesso = []
        self.folga = []
        
        self.excesso_c = []
        self.folga_c = []
        
    def readtxt(self, path):
    
        if self.matrix ==  []:
        
            arquivo = open(path,'r')

            aux = [linha.split() for linha in arquivo]

            self.num_var_p = int(aux[0][0])
            self.num_res_p = int(aux[0][1])
            self.obj_fun_p = aux[1]

            for i in range(len(aux[1])):
                self.obj_fun_p[i] = float(aux[1][i])

            for i in range(2,len(aux)):
                linha = []
                for j in range(len(aux[i])):
                    linha.append(int(aux[i][j]))
                self.matrix.append(linha)

            arquivo.close()
            
            print("===========================")
            print("Instância Lida com Sucesso!")
            print("===========================\n")
            
        
        
    def define_res(self, num_res, var, matrix, t):
        
        res = []
        
        if t.upper() == "PRIMAL":
        
            for i in range(num_res): ##Definindo as restricoes e multiplicando cada coeficiente com sua respectiva variavel
                res.append(self.solver.Constraint(matrix[i][-1], self.solver.infinity()))

                for j in range(len(var)):
                    res[i].SetCoefficient(var[j],matrix[i][j])
                    
        elif t.upper() == "DUAL":
            
            for i in range(num_res): ##Definindo as restricoes e multiplicando cada coeficiente com sua respectiva variavel
                res.append(self.solver_d.Constraint(-self.solver_d.infinity(), matrix[i][-1]))

                for j in range(len(var)):
                    res[i].SetCoefficient(var[j],matrix[i][j])
            
        return res
        
    def define_var(self, num_var, t):
        
        if t.upper() == "PRIMAL":

            var = []

            for i in range(num_var): ##Definindo as variaveis
                var.append(self.solver.NumVar(0, self.solver.infinity(), 'x' + str(i)))

        
        elif t.upper() == "DUAL":
            
            var = []

            for i in range(num_var): ##Definindo as variaveis
                var.append(self.solver_d.NumVar(0, self.solver_d.infinity(), 'x' + str(i)))

        return var
            
            
            
    def primal_solution(self):
        
        var = self.define_var(len(self.matrix[0])-1, "primal")
        res = self.define_res(len(self.matrix), var, self.matrix, "primal")
        
        self.var_p = var
        self.res_p = res
        if (self.solve(self.matrix, self.var_p, self.res_p, self.obj_fun_p, "primal") != 1):
            self.resultado_p, self.num_var_p, self.num_res_p, self.solucao_p = self.solve(self.matrix, self.var_p, self.res_p, self.obj_fun_p, "primal")
        else:
            self.matrix = []
            return 1
            ##self.resultado_p, self.num_var_p, self.num_res_p, self.solucao_p = self.solve(self.matrix, self.var_p, self.res_p, self.obj_fun_p, "primal")
            
    def dual_solution(self):
        
        if self.matrix != []:
        
            aux = np.transpose(self.matrix)

            self.dual_matrix = aux[0:self.num_var_p,:]
            self.dual_matrix = self.dual_matrix.tolist()

            for i in range(self.num_var_p):

                self.dual_matrix[i].append(self.obj_fun_p[i])
                
            self.num_var_dual = self.num_res_p
            self.num_res_dual = self.num_var_p
            
            var = self.define_var(len(self.dual_matrix[0])-1, "dual")
            res = self.define_res(len(self.dual_matrix), var, self.dual_matrix, "dual")
        else:
            print("Impossivel realizar a solução do dual, PRIMAL é inviável")
            return 1
        
        aux = aux.tolist()
        self.obj_fun_d = aux[len(self.dual_matrix)]
        
        self.var_d = var
        self.res_d = res
        if self.solve(self.dual_matrix, self.var_d, self.res_d, self.obj_fun_d, "dual") != 1:
            self.resultado_d, self.num_var_d, self.num_res_d, self.solucao_d = self.solve(self.dual_matrix, self.var_d, self.res_d, self.obj_fun_d, "dual")
        
        ##self.resultado_d, self.num_var_d, self.num_res_d, self.solucao_d = self.solve(self.dual_matrix, self.var_d, self.res_d, self.obj_fun_d, "dual")

    
    def solve(self, matrix, var, res, funobj, t):
        
        status = self.solver.Solve()
        status_d = self.solver_d.Solve()
        
        if status == self.solver.OPTIMAL and status_d == self.solver_d.OPTIMAL:
            
            objetivo = self.solver.Objective()
            objetivo_d = self.solver_d.Objective()
                
            if t.upper() == "PRIMAL":
                
                funcaoObjetivo =[]
                for i in range(len(var)):
                    funcaoObjetivo.append(objetivo.SetCoefficient(var[i], funobj[i]))

                objetivo.SetMinimization()
                self.solver.Solve()
                
                resultado = 0
                for i in range(len(var)):  ##Calculando o valor da funcao objetivo
                    resultado += funobj[i] * var[i].solution_value()
                sol_var = []
                for i in range(len(var)):
                    sol_var.append(var[i].solution_value())

                return resultado, self.solver.NumVariables(), self.solver.NumConstraints(), sol_var

            elif t.upper() == "DUAL":
                
                funcaoObjetivo =[]
                for i in range(len(var)):
                    funcaoObjetivo.append(objetivo_d.SetCoefficient(var[i], funobj[i]))

                objetivo_d.SetMaximization()
                self.solver_d.Solve()

                resultado = 0
                for i in range(len(var)):  ##Calculando o valor da funcao objetivo
                    resultado += funobj[i] * var[i].solution_value()
                sol_var = []
                for i in range(len(var)):
                    sol_var.append(var[i].solution_value())

                return resultado, self.solver_d.NumVariables(), self.solver_d.NumConstraints(), sol_var
        
        else:
            
            print("PROBLEMA INVIÁVEL")
            return 1
        
    def get_atributes(self, t):

        if self.matrix != []: ## Se for igual a [] o problema é inviavel
        
            self.get_excesso()
            self.get_folga()
            self.get_FC()
            
            if t.upper() == "PRIMAL":
                
                print(" ")
                print('Numero de variaveis do Primal =', self.num_var_p)
                print('Numero de restricoes do Primal =', self.num_res_p)
                print(" ")
                print('Solucao do Primal:\n')

                for i in range(self.num_var_p):
                    print('x{} = {:.2f}'.format(i+1, self.solucao_p[i]))
                    
                
                print("\nExcessos:")
                
                for i in range(len(self.excesso)):
                    
                    print("x{} = {:.2f}". format(i + self.num_var_p + 1, self.excesso[i]))
                    
                print("\nExcessos Complementares: ")
                for i in range(len(self.excesso_c)):
                    print("f{} = {:.2f}".format(i+1, self.excesso_c[i]))
                    
                print(" ")
                print('Valor otimo do Primal = {:.2f}'.format(self.resultado_p))
                
                
                
                
            elif t.upper() == "DUAL":
                
                print(" ")
                print('Numero de variaveis do Dual =', self.num_var_d)
                print('Numero de restricoes do Dual =', self.num_res_d)
                print(" ")
                print('Solucao do Dual:\n')

                for i in range(self.num_var_d):
                    print('y{} = {:.2f}'.format(i+1,self.solucao_d[i]))
                    
                self.get_folga()
                print("\nFolgas:")
                
                for i in range(len(self.folga)):
                    
                    print("y{} = {:.2f}". format(i + self.num_var_d + 1, self.folga[i]))
                    
                print("\nFolgas Complementares: ")
                for i in range(len(self.folga_c)):
                    print("f{} = {:.2f}".format(i+1, self.folga_c[i]))

                print(" ")
                print('Valor otimo do Dual = {:0.2f}'.format(self.resultado_d))
            
                
            else:
                print("TYPE ERROR")
                return 1
        
    def get_excesso(self):
        
        self.excesso = []

        for i in range(self.num_res_p): 

            ##soma = 0
            soma2 = 0
            for j in range(self.num_var_p):
                soma2 += self.matrix[i][j] * self.solucao_p[j]
                ##soma += matrix[i+2][j] * variaveis[j].solution_value()

            resultcerto = self.matrix[i][-1] - soma2
            self.excesso.append(resultcerto)
        
    def get_folga(self):

        
        self.folga = []

        for i in range(self.num_res_d): 

            soma = 0
            for j in range(self.num_var_d):
                soma += self.dual_matrix[i][j] * self.solucao_d[j]
                ##somadual += dual[i+2][j] * variaveisDual[j].solution_value()

            resultcertodual = self.dual_matrix[i][-1]  - soma
            self.folga.append(resultcertodual)
            
    def get_FC(self):
        
        excessovezes = []
        for i in range(len(self.excesso)):
            excessovezes.append(self.excesso[i] * self.solucao_d[i])

        folgavezes = []
        for i in range(len(self.folga)):
            folgavezes.append(self.folga[i] * self.solucao_p[i])

        self.excesso_c = excessovezes
        self.folga_c = folgavezes