//
//OBTENDO DADOS DO SISTEMA
//

//DADOS
    //COORDENADAS (x da base sempre 0)

    Coord=[0,.3,1;0,-0.3,1;1.2,0.15,0;1.2,-0.15,0]; //(coord x),(coord y),(engastado(s=1))

    //MATERIAIS

    Mat=[1,200000000000,0.3]; //(numero),(Modulo de elasticidade), (espessura)

    //BARRAS

    Bar=[1,2,3,4,1]; //(Nós da base),,(Nós do fim),,(Material) (SEMPRE DE CIMA PARA BAIXO)

    //FORÇA APLICADA NO FIM DA BARRA

    Forca=[3000000];

//
//CALCULANDO A EQUAÇÃO DA RETA REPESENTADA PELA BARRA
//
for c=1:10 // Cria o laço para 10 testes

n=c^2; // Aumenta o valor de n

for i=1:2
    b(i)=Coord(Bar(i),2); //CALCULA B DE AX+B
    a(i)=(Coord(Bar(i+2),2)-b(i))/Coord(Bar(i+2),1);//CALCULA A DE AX+B
end

F(n+1,1)=Forca;
//
//CALCULANDO A DIVISÃO DE BARRAS 
//

Div=(Coord(Bar(3),1)/n);
for j=1:2
    y(j,1)=Coord(Bar(j),2);
    for i=1:n
        y(j,i+1)=i*Div*a(j)+b(j); //CALCULA CADA VALOR DE Y
        ym(j,i)=(y(j,i)+y(j,i+1))/2; // CALCULA CADA MÉDIA DE Y
    end
end
for i=1:n
    yd(i)=ym(1,i)-ym(2,i); //CALCULA A ALTURA TOTAL DE CADA DIVISAO
end

//
//CALCULANDO Ks
//

for i=1:(length(yd))
    Ks(i)=(Mat(Bar(5),2)*(Mat(Bar(5),3)*yd(i)))/Div; // CALCULA UM K PARA CADA DIVISAO
end

//
//CALCULANDO A MATRIZ FINAL DE K
//

K=zeros(n+1,n+1);
K(1,1)=Ks(1);
K(n+1,n+1)=Ks(n);
K(n+1,n)=-Ks(n);
K(n,n+1)=-Ks(n);
for i=2:n
    K(i,i)=(Ks(i)+Ks(i-1));
    K(i,i-1)=-Ks(i-1);
    K(i-1,i)=-Ks(i-1);
end

//
// RESOLVENDO O SISTEMA LINEAR DE DESLOCAMENTO
//

tmp2=K(1,:);
tmp=K;
tmpF=F;

tmp(1,:)=[]; //Exclui linhas
tmp(:,1)=[]; //Exclui colunas
tmpF(1)=[]; //Exclui a primeira linha da força

u2=(linsolve(tmp,tmpF))*-1; //RESOLVE O SISTEMA LINEAR u=K*F
u(2:n+1)=u2; //CRIA UM VETOR DE DESLOCAMENTOS DE TODOS OS NOS

U(c)=sum(u);
N(c)=n;

clear K Ks tmpF yd y u2 u a b u2 tmp tmp2 ym F j i Div 

end

plot(N,U) //plota o gráfico Numero de nós x Deslocamento
