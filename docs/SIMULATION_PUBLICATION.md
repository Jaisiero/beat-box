# Publicación de simulación y TLAS

## Cambios

Se elimina el `device.wait_idle()` entre actualizar las instancias y construir el
TLAS. Los dos graphs conservan el mismo `TaskBuffer` externo de instancias:
la escritura compute deja registrada su cola productora y el graph de AS espera
el último envío de esa cola mediante los semáforos internos de Daxa. El render
consume del mismo modo el `TaskTlas` publicado. No se presupone que el orden FIFO
por sí solo haga visibles las escrituras.

Se retiran el semáforo `sim_done_tsem`, sus contadores y spans: en la revisión
vendorizada de Daxa 3.6, `TaskGraph::submit` ignora `TaskSubmitInfo`. Por tanto,
los campos `additional_*_timeline_semaphores` del código anterior no aportaban
sincronización. La dependencia efectiva ya era la de recursos externos.

Las esperas inmediatamente posteriores a `simulate()` ahora esperan el último
envío de `QUEUE_COMPUTE_0`, donde se ejecuta el solver. No esperan globalmente
las otras colas. Se mantienen las esperas previas a la simulación y las fronteras
de vida útil de recursos, cambios de escena, readback y fragmentación que todavía
necesitan el protocolo actual. Esta modificación no elimina toda comunicación
CPU/GPU ni cambia el algoritmo físico.

El BLAS de depuración lee geometría del buffer de nodos LBVH. Se añade ese buffer
como `BUILD_READ` al task de construcción y al registro de recursos de su graph.

## Restricción del compilador de graphs

Se probaron un graph combinado y un graph combinado con dos submits. Ambos
fallaron durante `TaskGraph::complete`, antes de ejecutar la simulación. GDB
localizó un `ArenaDynamicArray8k<TaskBarrier>` con allocator nulo al insertar la
barrera del buffer `blas_instance_data`, entre escritura compute y lectura AS.
No se modifica Daxa en esta rama: se conservan los dos graphs existentes,
sin la espera de CPU intermedia. La causa interna exacta requiere un caso mínimo
independiente; no se atribuye el fallo a Vulkan ni a una carrera de GPU.

## Validación

- Build Release y cinco tests de CTest correctos.
- Fixture de rotura: 900 pasos con AVBD y 900 con TGS.
- F7: 1800 pasos con AVBD y 1800 con TGS.
- Los cuatro CSV y todos los checkpoints DET coinciden exactamente con los
  controles de PR25. Vulkan synchronization validation no reporta errores;
  la fixture también activa la verificación de pools.
- Interacción a 3840×2160 con validación Vulkan: roturas mediante drag/release
  en F9, BVH y reset, pausa/acumulación, F5/F7/F9, cámara y resize. Sin errores
  reportados. Capturas `ui-fracture.png` y `ui-bvh.png`.
- F6 A/B contra el ejecutable anterior, tres repeticiones de 600 pasos por
  solver y variante. Todos los CSV y checkpoints coinciden dentro de cada
  solver. Sin validación Vulkan ni profiling para medir rendimiento.

| Solver | Antes, ms/frame | Después, ms/frame | Reducción |
| --- | ---: | ---: | ---: |
| AVBD | 4,450 | 4,024 | 9,6 % |
| TGS | 6,034 | 5,804 | 3,8 % |

Medianas de tiempo de pared entre checkpoints 1 y 600, dividido entre 599.
Incluyen simulación, publicación, render y coste del harness a 860×640;
**no son tiempos GPU exclusivos del solver ni cifras de render a 4K**.
Hardware: RTX 4090, LXC 110. Es una medición local, no una garantía para otras
escenas, resoluciones o arquitecturas.

Evidencia: `/root/beat-box/work/publication/`, incluidos `checks.log`,
`timings.json`, logs/CSV de cada repetición y los backtraces de los intentos
de combinar graphs.
