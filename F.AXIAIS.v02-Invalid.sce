//
//OBTENDO DADOS DO SISTEMA (CONSIDERANDO APOIOS NAS DUAS EXTREMIDADES)
//

//DADOS

Coord=[0;0.5;0.8;1.1];

Mat=[1,25000000,0.04 ; 2,200000000,0.0025];

Bar=[1,2,1;2,3,2;3,4,1];

Forca=[0;-30;70;0]

//CALCULANDO Ks

for i=1:(length(Bar)/3)
    Ks(i)=(Mat(Bar(i,3),2).*Mat(Bar(i,3),3))/(abs(Coord(Bar(i,1))-Coord(Bar(i,2))));
end

//
//CALCULANDO A MATRIZ FINAL DE K
//
tmp=size(Bar);
nb=tmp(1);
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
// RESOLVENDO O SISTEMA LINEAR DE DESLOCAMENTO
//

tmp=K(2:nb,2:nb);
u=linsolve(tmp,Forca(2:nb));

//
// RESOLVENDO REAÇÕES DE APOIO
//

Rx1=(K(1,2)*u(1))+Forca(1);
Rx2=(K(nb+1,nb)*u(nb-1))+Forca(nb+1);

//
//RESOLVENDO AS FORÇAS NOS NÓS (INVALID INDEX)
//

j=1;
for i = 1:(nb-1)
    pause
    KF(i,i)=Ks(i+1);
    KF(i+1,i+1)=Ks(i+1);
    KF(i,i+1)=-Ks(i+1);
    KF(i+1,i)=-Ks(i+1);
    FN(j)=abs(KF(i,1).*u(i)+KF(i,2).*u(i+1));
    FN(j+1)=abs(KF(i+1,1)*u(i)+KF(i+1,2)*u(i+1));
    j=j+1;
    i=i+1;
    clear KF;
end

//
//APRESENTANDO RESULTADOS
//

mprintf("Deslocamentos em ordem crescente de nós: \n 0")
disp(u)
mprintf("0 \n")
mprintf("Forças de reação no Nó 1: %g \n",Rx1)
for i=1:nb-1
    mprintf("Forças no Nó %i: %g \n",i,FN(i))
end
mprintf("Forças de reação no Nó %i: %g \n",nb+1,Rx2)




