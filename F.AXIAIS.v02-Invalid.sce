//
//OBTENDO DADOS DO SISTEMA (CONSIDERANDO APOIOS NAS DUAS EXTREMIDADES)
//

//DADOS

Coord=[0, 1;0.5, 0;0.8, 0;1.1, 1]; //O número após a coordenada X indica a existência ou não de apoio

Mat=[1,25000000,0.04 ; 2,200000000,0.0025];

Bar=[1,2,1;2,3,2;3,4,1]; //Nós abrangidos e material *EM ORDEM*

Forca=[0;-30;70;0]

//CALCULANDO Ks

for i=1:(length(Bar)/3)
    Ks(i)=(Mat(Bar(i,3),2).*Mat(Bar(i,3),3))/(abs(Coord(Bar(i,1),1)-Coord(Bar(i,2),1)));
end

//
//CALCULANDO A MATRIZ FINAL DE K
//
tmp=size(Bar);
nb=tmp(1);
clear tmp;
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

tmp=K;
tmp2=K;
j=0;
j2=0;
for i=1:(nb+1)
    if Coord(i,2)==1
        j=j+1;
        Sub(j)=i;
    else
        j2=j2+1;
        Sub2(j2)=i;
    end
end
x=0;
tmpF=Forca;

while(j2>=1)
    tmp2(Sub2(j2),:)=[];
    j2=j2-1;
end

for i=1:j
    tmp((Sub(i)-x),:)=[];
    tmp(:,(Sub(i)-x))=[];
    x=x+1;
end

tmpF(Sub)=[];

u2=linsolve(tmp,tmpF);
u([Sub2])=u2*-1;
u([Sub])=0;

//
// RESOLVENDO REAÇÕES DE APOIO
//

for i=1:j
    Rx(i)=(tmp2(i,:)*u)+Forca(Sub(i));
end

//
//RESOLVENDO ESFORÇOS INTERNOS
//

c=1;
for i=1:nb
    if (Coord(Bar(i,1),2)==1 & Coord(Bar(i,2),2)==1)
        FBar(i)=0;
    else if (Coord(Bar(i,1),2)==1)
        FBar(i)=Rx(c);
        c=c+1;
    else if(Coord(Bar(i,2),2)==1)
        FBar(i)=Rx(c)*-1;
        c=c+1;
    else
        FBar(i)=(Ks(i)*u(i))-(Ks(i)*u(i+1));
    end
end
end
end

//
//APRESENTANDO RESULTADOS
// 

mprintf("Deslocamentos em ordem crescente de nós: \n")
disp(u)
for i=1:j
    mprintf("\n Forças de Reação no Nó %i: %g \n",Sub(i),Rx(i))
end

for i=1:nb
    mprintf("\n Esforços internos na Barra %i: %g  ",i,abs(FBar(i)))
    if FBar(i)>0
        mprintf("Tração \n")
    else if FBar(i)<0
        mprintf ("Compressão \n")
    end
end
end
