docker images

docker run --docker_image 

Esse comando roda o docker como um script, executando o software contido nele.

Para poder ter acesso ao terminal e digitar comandos lá, é necessário adicionar a flag -t 

-i: Interactive -> Posso interagir
-t: TTY -> me dá acesso ao terminal

docker image rm/rmi -f image_name/ID

a flag -f força a remoção

docker pull

docker ps
docker ps -a

-a: mostra todos os containers (ativos ou inativos)

```
docker container stop container_name/ID
```

```
docker container start -i container_name/ID
```
retorna para o container que foi criado anteriormente (com as alterações realizadas no container, pastas, arquivos e etc)
a flag é o -i mesmo e não -it. porque o start não cria um novo terminal interativo do zero

para sair do container: Exit ou ctrl + d

Se usar o comando docker run na mesma imagem, outro container será criado, com nome diferente do anterior, mesmo com as imagens sendo iguais. É possível instanciar quantos containers for desejado com a mesma imagem

```
docker rm container_name/ID
```
Para remover um container (e não a imagem)

docker container prune
e para remover todos os containers *inativos* ao mesmo tempo

--rm: flag usada para deletar automaticamente o container toda vez que ele ficar inativo
--name: flag para adicionar o nome desejado para o container


```
docker exec -it container_name/ID /bin/bash
```
Para abrir outro terminal de um container ativo (é necessário colocar o /bin/bash para abrir um shell bash dentro do container, ou seja, um terminal interativo)

docker exec -it container_name/ID ls
Também é possível executar um comando dentro do container (nesse caso o comando para listar todos os itens com o ls)

## Dockerfile 

**Fazer analogia com o DNA**


Dockerfile: "Documento" em que é colocado todas as instruções para gerar a imagem desejada

O que será instalado nesse docker vai depender da sua aplicação (por exemplo alguma biblioteca específica do ROS)

sempre começa com um from (a ideia é que ele vai aproveitar instruções de outras imagens)

run é usado para passar as funcionalidades que você deseja ter

O que for adicionado aqui é parte intrínseca da imagem, sendo muito complicado de remover.

e o Copy copia os arquivos de uma pasta (diretórios, arquivos) para dentro do docker

Ex: Tenho uma pasta de configurações com um arquivo dentro e eu preciso copiar essa pasta para dentro do meu docker e armazena na pasta desejada.
```
COPY config/ /site_config/
```
ele diz que não é necessário o sudo dentro do docker porque ele roda como root -> Estranho!!

para buildar o dockerfile

```
docker build -t image_name
```

```
FROM ros:noetic

RUN apt-get update && apt-get install -y nano && rm -rf /var/lib/apt/lists/*

COPY config/ /site_config/

```
o From vai aproveitar a imagem já criada do ros noetic
o run desse exemplo é usado para instalar o nano.
o COPY é usado para copiar uma pasta do seu computador para dentro da imagem (para buildar, é necessário incluir essa pasta, ela fica como dependência). Nesse exemplo, o Dockerfile está junto com a pasta que eu quero copiar e por isso foi usado o ponto final. Não é possível buildar com arquivos fora do mesmo diretório.

```
docker build -t image_name .
```
a ideia de remover /var/lib/apt/lists/* se dá devido ao comando update armazenar uma lista de atualizações nessa pasta, podendo gerar lixo desnecessário e até mesmo ocasionar erro no build


## Volume/Bind Mount

Usado para compartilhar/armazenar arquivos entre o container e a máquina

Para criar um volume (pasta na qual serão salvos os arquivos dentro do container)

```
docker run -it -v nome_do_volume:/caminho_container my_image

```
A pasta a qual esses arquivos estão armazenados no host estpa localizada em /var/lib/docker/volumes/nome_do_volume

Por algum motivo essa pasta precisa de permissão do root

Para criar um bind mount (é criado uma pasta dentro do host e uma dentro do container e todos os arquivos inseridos lá serão compartilhados entre os dois)

```
docker run -it -v caminho_host:/caminho_container my_image
```
Um arquivo criado dentro do container (com permissão de root) sempre vai precisar de acesso de root para ser aberta no host

## User

Como foi visto anteriormente, criar um arquivo dentro do container, ao ser acessado pelo host, ele terá permissão de root, uma vez que o container é definido com permissão de root por padrão. 

Com isso, a ideia é configurar o docker para estar no mesmo nível de permissão do host para permitir que arquivos criados dentro do container tenham permissão de serem acessados pelo host.

A ideia é que, como o host e o usuário dentro do container possuem o mesmo UID e GID, eles são iguais e possuem as mesmas permissões (rw).

Em alguns casos pode ser necessário que o nome do usuário seja igual, mas na maioria das aplicações, desse forma funciona

Para isso, foram incluídos no Dockerfile o seguinte comando

```
ARG USERNAME=ros

ARG USER_UID=1000

ARG USER_GID=$USER_UID
  
#create a non-root user
  
RUN groupadd --gid $USER_GID $USERNAME \

&& useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \

&& mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config
```

A explicação:

### **1. `groupadd --gid $USER_GID $USERNAME`**

- **`groupadd`** cria um novo grupo no container.
    
- **`--gid $USER_GID`** define manualmente o **Group ID** para esse grupo (vem de uma variável passada no build ou no `docker run`).
    
- **`$USERNAME`** é o nome do grupo (mesmo do usuário, para manter consistência).
    
- Isso garante que o grupo no container tenha o **mesmo GID** do usuário na máquina host, evitando problemas de permissão em volumes compartilhados.
    

---

### **2. `useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME`**

- **`useradd`** cria um novo usuário.
    
- **`-s /bin/bash`** define o shell padrão do usuário como **bash**.
    
- **`--uid $USER_UID`** força o **User ID** a ser o mesmo que o do host (igual motivo do GID).
    
- **`--gid $USER_GID`** associa o usuário ao grupo criado antes.
    
- **`-m`** cria automaticamente o **home directory** (`/home/$USERNAME`).
    
- Assim, arquivos criados dentro do container terão **o mesmo UID/GID no host**, evitando precisar de `chmod` depois.
    

---

### **3. `mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config`**

- **`mkdir /home/$USERNAME/.config`** cria a pasta `.config` no home do novo usuário (muitos programas gráficos e CLI usam essa pasta para salvar configs).
    
- **`chown $USER_UID:$USER_GID ...`** muda o dono dessa pasta para o usuário e grupo criados, garantindo que ele possa escrever ali.

Para rodar com a nova configuração do user, é necessário passar o usuário a qual vc deseja entrar no container (como criamos o usuário ros, precisamos passar ele)
``
```
docker run -it --user ros -v ~/volumes:container_volume my_image
```
- Se não passar, por padrão ele cria como root

- Se o usuário for diferente do que ele espera (definido no dockefile), ele dá erro no run


## SUDO 

Agora que o nosso usuário dentro do container perdeu as permissões de root, precisamos definir acesso ao sudo.

Para isso, basta adicionar a seguinte linha no Dockerfile

Esse comando permite que o usuário definido na variável `$USERNAME` rode qualquer comando como root dentro do container sem precisar digitar senha, mantendo as permissões corretas.

```
RUN apt-get update \ && apt-get install -y sudo \ && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \ && chmod 0440 /etc/sudoers.d/${USERNAME} \ && rm -rf /var/lib/apt/lists*
```

## NETWORKING

A ideia é que o container e o host compartilhem a mesma configuração de rede

```
docker run -it --user ros --network=host --ipc=host -v ~/volumes:/container_volume my_image
```
 **--network=host**

- Faz o container **usar diretamente a rede do host**, sem criar a rede isolada padrão do Docker.
**--ipc=host**

- Faz o container compartilhar o **espaço de memória IPC (Inter-Process Communication)** do host.

## Entrypoint/CMD

O ENTRYPOINT no Docker é o comando principal que sempre será executado quando o container iniciar.

## Diferença para `CMD`

ENTRYPOINT é o **comando fixo** que o container sempre executa; CMD são **os argumentos** ou o comando default, mas podem ser sobrescritos.

- **ENTRYPOINT** → Define **o executável fixo** do container.
    
- **CMD** → Define **argumentos padrão** (ou, se não tiver ENTRYPOINT, define o comando inteiro a rodar).


Um script para o entrypoint

```
#!/bin/bash

set -e

source /opt/ros/noetic/setup.bash

echo "Provided arguments: $@"

exec $@

```

No dockefile

```
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

CMD ["bash"]
```
## GUI 

Para poder usar o GUI no container, é preciso que o container tenha permissão de acessar o X


Se estiver rodando o container com as configurações/permissões do host (já tem permissão de usar o X11)

xhost +   (comando no host)
para dar permissão para todos os usuários

xhost +local:
para dar permissão para usuários locais

ou 

xhost +local:root
para dar permissão para o usuário específico

Para tirar as permissões, basta trocar o + pelo - nos comandos

Em seguida, é preciso compartilhar o socket do X11

--env=DISPLAY no `docker run` significa que você está **passando uma variável de ambiente para dentro do container**.

- **`--env`** (ou `-e`) → define uma variável de ambiente no container.
    
- **`DISPLAY`** → é o nome da variável que o **X11** (sistema de janelas do Linux) usa para saber **onde enviar as janelas gráficas**.
o Docker pega **o valor do `DISPLAY` do seu host** e o define **dentro do container**

Alguns outros comandos podem ser encontrados no vídeo de referência (parte 3) - Articulated robotics
```
docker run -it --user ros --network=host --ipc=host -v ~/volumes:/container_volume -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY my_image
```

Com esse comando, as aplicações gráficas já estão funcionando dentro do container


## Configurar o Bashrc


COPY bashrc /home/&{USERNAME}/.bashrc

source /opt/ros/noetic/setup.bash
# Devices in Docker

No linux, tudo é arquivo, incluindo a maioria dos dispositivos

Mesmo que eu tente compartilhar os arquivos usando Bind Mount, não vai funcionar -> o container precisa saber que aquele arquivo se trata de um device

Esse comando permite o uso do joystick dentro do container, mas possui algumas limitações (se o dispositivo não estiver conectado na hora de rodar o container ou se trocar o id, vai dar problema)
```
docker run -it --user ros --network=host --ipc=host -v ~/volumes:/container_volume -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY --device=/dev/input/js0 my_image

```
Para superar os problemas, a ideia é criar um Bind Mount e passar a informação de que ele é um dispositivo com a flag --device

```
docker run -it --user ros --network=host --ipc=host -v ~/volumes:/container_volume -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY -v /dev/input:/dev/input --device-cgroup='c 13:* rmw' my_image
```
### Sobre `--device-cgroup-rule='c 13:* rmw'`

- Esse parâmetro é usado para permitir acesso a dispositivos **no nível de regras de cgroup**, e é útil quando você quer dar acesso a **todos os dispositivos com major number 13** (que são dispositivos de entrada como joysticks e mouses).
    
- `c` significa “character device” (dispositivo de caracteres), `13` é o major number, e `*` significa “todos os minor numbers”.
* rmw: read, mknod, write


Os dispositivos podem ser:

- **Caractere (`c`)** → Transferem dados **byte a byte** (stream).  
    Exemplos: portas seriais (`/dev/ttyUSB0`), teclado, mouse, LiDAR, câmera USB, joystick.
    
- **Bloco (`b`)** → Transferem dados **em blocos** (com cache/buffer).  

- **Major number** → diz **qual driver** controla o dispositivo.
    
- **Minor number** → identifica **qual dispositivo** específico, dentro daquele driver.
A tabela mostrando os principais dispositivos do tipo c

| Major | Tipo | Dispositivo(s) |
|-------|------|----------------|
| 4     | c    | Portas seriais padrão (`/dev/ttyS*`) |
| 13    | c    | Dispositivos de entrada (`/dev/input/*` → teclado, mouse, joystick) |
| 81    | c    | Câmeras de vídeo USB (`/dev/video*`) |
| 188   | c    | Conversores USB-serial (`/dev/ttyUSB*`) |
| 189   | c    | Dispositivos USB genéricos |
| 226   | c    | GPU DRM (renderização, como `/dev/dri/*`) |

Para dar acesso a todos os dispositivos do host, incluindo discos e outros hardwares (não recomendado!)
```
-v /dev:/dev --device-cgroup-rule='c *:* rmw'
```
O recomendado é dar permissão somente para o driver que vai ser utilizado (mas infelizmente nem sempre é possível)
### Câmera

No exemplo do vídeo, a câmera se enquadrou no major 189 (dispositivos USB genérico)

```
docker run -it --user ros --network=host --ipc=host -v ~/volumes:/container_volume -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY -v /dev/bus:/dev/bus --device-cgroup='c 189:* rmw' my_image
```
Nesse exemplo, não é possível atribuir permissão somente para a câmera pois, a cada vez que ela for retirada e recolocada,. ela muda o seu endereço 

Existe um major específico para câmeras (as vezes você pode estar tentando atribuir para o major genérico mas o sistema jogou a câmera nesse major específico)
```
--device-cgroup-rule='c 81:* rmw'
```
### Serial Device

Nesse caso, mesmo com privileged mode, talvez não funcione

Os dispositivos estão no grupo dialout e somente usuários nesse grupo tem permissão de rw: read and write

para isso, basta incluir o usuário como pertencente a esse grupo (pode ser diretamente no Dockerfile ou no terminal dentro do container)

```
usermod -aG dialout ${USERNAME}
```

Mas a ideia é não usar o privileged mode porque ele concede permissão para o container modificar todo e qualquer hardware do host -> Pode ser perigoso!!!

Se você souber qual dispositivo você quer conceder permissão ou se só tiver um conectado no computador

```
docker run -it --user ros --network=host --ipc=host -v ~/volumes:/container_volume -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY --device=/dev/ttyACM1 my_image
```

Normalmente tem mais de um dispositivo conectado a porta serial e, por isso, a forma anterior não é o mais apropriado

```
ls -l /dev/serial/by-id

```
Para listar os dispositivos USBS conectados

```
lsusb
```

Para listar todos por tipo

```
ls -l /dev | grep " 189,"
```

```
docker run -it --user ros --network=host --ipc=host -v ~/volumes:/container_volume -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY -v /dev:/dev --device-cgroup-rule='c *:* rmw' my_image
```
Ainda preciso melhorar essas permissões de acesso ao device group. Por enquanto tá permitindo tudo