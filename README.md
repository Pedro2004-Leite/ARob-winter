# ARob-winter

## O que é este curso

Aeronaves Robotizadas / Unmanned Aerial Vehicles (UAVs), MEAer, Instituto Superior Técnico.
Na sua essência, o curso ensina a construir o ciclo de controlo completo de um quadrotor.
Isso significa: modelar a dinâmica de corpo rígido do veículo, desenhar controladores (lineares e depois não-lineares) capazes de seguir uma trajetória, estimar o estado real a partir de sensores ruidosos e sem GPS, e por fim planear e seguir caminhos geométricos entre waypoints.
Este ciclo - Modelação → Controlo → Sensores/Estimação → Guiamento - repete-se em cada capítulo teórico e é reproduzido, na prática, nos três laboratórios com o quadrotor Parrot AR.Drone.
A referência bibliográfica principal é Beard & McLain, *Small Unmanned Aircraft: Theory and Practice* (Princeton University Press, 2012).

## Conteúdo deste repositório

- `lectures-22-23/` - slides das aulas da edição 2022/23 (capítulos 0 a 9), com `COURSE_NOTES.md` a servir de base de conhecimento profunda sobre toda a teoria.
- `labs-25-26/` - enunciados e devkits Simulink dos três laboratórios da edição 2025/26, com `LAB_NOTES.md` a documentar a componente prática.

## Para ir mais fundo

- [MIT 6.832 - Underactuated Robotics](https://underactuated.mit.edu/) (Russ Tedrake) cobre praticamente a mesma espinha dorsal teórica deste curso: dinâmica de corpo rígido subactuado, LQR, Lyapunov, controlo não-linear e um capítulo inteiro dedicado a quadrotores.
  Acesso direto, totalmente gratuito: o livro completo e as gravações em vídeo de todas as aulas estão no próprio site.
- [Steve Brunton - Control Bootcamp](https://www.youtube.com/@Eigensteve) (Universidade de Washington), canal de YouTube gratuito com playlists dedicadas a espaço de estados, LQR, filtro de Kalman e sistemas não-lineares - excelente complemento visual aos capítulos 4, 6 e 7 deste curso.
