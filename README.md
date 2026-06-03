# Grasshopper Parametric Terrain & Accessible Paths

Repositório contendo o script base em Python (RhinoCommon) para geração de terrenos ondulados (montículos/mounds) com a inserção de caminhos com controle automático de inclinação acessível (máximo de 8,33% ou 1:12) e recorte automático de superfícies para evitar sobreposição (z-fighting).

## 🚀 Funcionalidades

- **Deformação Gaussiana Suave**: Gera elevações montanhosas naturais a partir de pontos de controle (atratores).
- **Atenuação de Borda (Smoothstep)**: Garante que o relevo retorne à altura zero exatamente no limite definido pela curva base, evitando quebras secas.
- **Rampa Acessível (Cálculo de Ida e Volta)**: Analisa a curva de eixo do caminho e corrige as alturas ponto a ponto para respeitar o limite máximo de inclinação de **8,33%** (NBR 9050).
- **Recorte Automático 2D/3D (Puzzle-Fit)**: Recorta a área do caminho diretamente de dentro do terreno usando operações booleanas 2D estáveis, eliminando o *z-fighting*.

## 📁 Estrutura de Arquivos

- `grasshopper_terrain_path_base.py`: Código-fonte principal para colar no componente GhPython.

## 🛠️ Como Usar no Grasshopper

1. Insira um componente **Python 3** ou **GhPython** na tela.
2. Adicione os seguintes parâmetros de **Input**:
   - `C` (Type Hint: `Curve`, Item Access): Limite externo da grama.
   - `P` (Type Hint: `Point3d`, List Access): Pontos dos picos elevados.
   - `H` (Type Hint: `float`, Item/List Access): Altura máxima dos picos.
   - `R` (Type Hint: `float`, Item/List Access): Raio de influência de cada pico.
   - `F` (Type Hint: `float`, Item Access): Distância de transição suave na borda.
   - `N` (Type Hint: `int`, Item Access): Resolução da grade (ex: 50).
   - `CRV` (Type Hint: `Curve`, Item Access, Opcional): Eixo central do caminho.
   - `W` (Type Hint: `float`, Item Access, Opcional): Largura do caminho (ex: 2.0).
   - `B` (Type Hint: `float`, Item Access, Opcional): Largura do talude de transição.
   - `MAX_S` (Type Hint: `float`, Item Access, Opcional): Inclinação máxima (Padrão: 0.0833).
3. Adicione os seguintes parâmetros de **Output**:
   - `S`: Superfície NURBS do terreno recortado.
   - `M`: Malha (Mesh) do terreno recortada.
   - `PM`: Malha (Mesh) 3D do caminho.
   - `PC`: Curvas de borda do caminho.
4. Cole o código de `grasshopper_terrain_path_base.py` no editor do componente.
