# TODO - Trabalho Final da disciplina SSC0714

**Tarefas:**

- [ ] Substituir Gyro pela bússola fornecida no arquivo `GPS-Compass.ttt`

- [ ] Programa 1: Gera uma representação completa do cenário e realiza o planejamento global de trajetória à priori
   - Representação do Mapa pode ser: 
     	- Mapas de Ocupação (*Grid*)
     	- Mapas Topológicos (Gerados automaticamente, ou então, desenhados a mão)
   - Gerar uma trajetória direta ao objetivo (salvar `waypoints.txt`)

- [ ] Programa 2: Navegação por Waypoints + Comportamento Reativo (Desvio de Obstáculos)
  - Waypoints são definidos a priori pelo algoritmo de planejamento



**Perguntas:**

1. Que linguagens podemos usar?

   Python, Matlab, C, etc.

2. Disciplina de algoritmos de estimação será oferecida no próximo semestre?

   Pergunte ao Prof. Denis, ele é o professor responsável por dar este conteúdo
   http://wiki.icmc.usp.br/index.php/SSC-5880(Denis)

3. Como é feito o planejamento de trajetória em quando não conhecesse o mapa do ambiente? *Frontier-based Navigation*

   Pode-se utilizar um algoritmo de campos potencias onde regiões de fronteira de zonas desconhecidas (áreas cinzas) tornam-se pontos de atração, o que faz com que o robô dirija-se até elas para explorá-las.



**Ideia:**

*Fase inicial:* A construção do mapa por exploração e mapeamento (Frontier-based Navigation? + GMapping?)
*Fase produção:* Usar o mapa construído (Navegação por WayPoints?)