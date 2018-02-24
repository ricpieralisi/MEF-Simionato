//
//OBTENDO DADOS DO SISTEMA (CONSIDERANDO APOIOS NAS DUAS EXTREMIDADES)
//

nb=input("Número de Barras")
for i=1:nb
    mprintf("Para a barra %i ",i)
    E(i)=input("Digite a Tensão máxima da barra ");
    A(i)=input("Digite a área da seção da barra ");
    L(i)=input("Digite o comprimento da barra ");
    Ks(i)=((E(i).*A(i))/L(i));
end

for i=2:nb
    mprintf("Forças axiais no nó %i ",i)
    F(i)=input(":");
end

//
//CALCULANDO A MATRIZ FINAL DE K
//

K=zeros(nb+1,nb+1);
K(1,1)=Ks(1);
K(nb+1,nb+1)=Ks(nb);
K(nb+1,nb)=-Ks(nb);
K(nb,nb+1)=-Ks(nb);
for i=2:nb
    K(i,i)=(Ks(i)+Ks(i-1));
    K(i,i-1)=-Ks(i-1);
    K(i-1,i)=-Ks(i-1);
end

//
// RESOLVENDO O SISTEMA LINEAR
//

tmp=K(2:nb,2:nb);
u=linsolve(tmp,F(2:nb));

//
// RESOLVENDO REAÇÕES DE APOIO
//

Rx1=K(1,2)*u(1);
Rx2=K(nb+1,nb)*u(nb-1);

//
//APRESENTANDO RESULTADOS
//

mprintf("Deslocamentos em ordem crescente de nós: 0")
disp(u)
mprintf("0 \n")
mprintf("Forças de reação no Nó 1: %g \n",Rx1)
mprintf("Forças de reação no Nó 2: %g \n",Rx2)




