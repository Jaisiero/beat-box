# Revisión transversal del proyecto — 2026-09-06

Base: merge `dc0e15d` de PR #24. Ese merge termina en `fbb88a4`: no contiene
`90c5fe4`, la optimización posterior de TGS. Esta rama incorpora ese commit como
`633db0f`, además de las correcciones descritas aquí.

## Alcance y criterio

Revisión de creación/destrucción de managers, publicación de simulación y AS,
render/path tracing, cámara/entrada/GUI, memoria/pools, física/SDF/fractura,
compilación/dependencias y herramientas de prueba. Se inspeccionaron las rutas
principales y sus contratos compartidos, y la implementación instalada de Daxa
en los puntos de sincronización relevantes. No es una demostración exhaustiva de
ausencia de errores en cada shader o intercalado de GPU.

Las pruebas dinámicas se realizan en el LXC 110, RTX 4090/Linux. No validan otra
arquitectura de GPU, el ejecutable nativo de Windows ni todas las topologías SDF.
Se distinguen defectos confirmados por código, resultados medidos e hipótesis que
requieren experimentos. No se han cambiado tolerancias ni aritmética del solver.

## Correcciones incluidas

### P1: tamaño incorrecto del BLAS de depuración LBVH

`src/acceleration_structure_manager.cpp:804` utilizaba
`2 * current_primitive_count - 1`. El shader `passes/bvh.slang` construye una hoja
por cuerpo: el número correcto es `2 * current_rigid_body_count - 1`. En una
escena de cajas simples coincidían; un cuerpo voxel tiene varias primitivas de
render y rompe esa equivalencia. Al activar L, el BLAS podía leer nodos que no se
habían construido y superar la capacidad de `MAX_LBVH_NODE_COUNT`.

Se usa el número de cuerpos y se evita el cálculo para cero cuerpos. El cambio
solo afecta a la visualización BVH, no al árbol usado por la física.

### P1: escritura de cámara sobre un buffer que puede seguir en uso

`src/camera_manager.hpp:52` escribía matrices directamente en un único buffer
mapeado. El render lee ese buffer tanto desde ray tracing como desde la GUI.
Las rutas pausadas/de reposo no garantizan una espera de la lectura anterior antes
de cada escritura de CPU; adquirir otra imagen del swapchain no es una garantía
de vida útil para ese buffer compartido. Una barrera posterior de GPU no ordena
retroactivamente la escritura realizada por CPU.

La cámara ahora conserva un valor CPU. La tarea de upload toma un snapshot en el
allocator de staging de Daxa y copia a un buffer de dispositivo. La configuración
del render usa el mismo mecanismo: desaparecen los dos buffers host reutilizados
por paridad. Su buffer de dispositivo también conserva una identidad estable,
para mantener el estado de dependencias entre ejecuciones. Accesos TRANSFER_WRITE
y lecturas de shaders permiten a Daxa ordenar las copias. No se añade wait_idle.

Esta corrección se justifica por el contrato de acceso; no se atribuyen a ella
los antiguos píxeles negros ya corregidos ni se afirma haber reproducido un fallo
visual específico de esta carrera.

### P2: polling activo al minimizar

`src/window_manager.hpp:150` detecta también GLFW_ICONIFIED y espera eventos hasta
100 ms mientras no puede renderizar. Antes, una ventana iconificada que conservaba
sus dimensiones no entraba en la condición de tamaño cero.

Prueba de tres segundos por ejecutable, proceso completo, expresada como porcentaje
de un núcleo: **33,0% antes, 0,33% después**. Es una observación puntual de CPU
minimizada, no una mejora de FPS ni una mediana de benchmarks.

## Prioridades pendientes

| Prioridad | Área y evidencia | Mejora concreta y validación requerida |
| --- | --- | --- |
| P1 | Semáforos explícitos: `src/gpu_context.hpp:46`, `TaskGraph::submit` de Daxa | Eliminar la falsa suposición de que los semáforos adicionales se aplican. Establecer un contrato comprobado de publicación/consumo antes de quitar esperas. |
| P1 | Esperas globales: `src/renderer_manager.cpp:608`, `:611`, `:784`; `AccelerationStructureManager::update_TLAS` | Separar finalización de cada slot y publicación de AS; sustituir esperas globales por dependencias/esperas del submit necesario. Medir CPU y GPU por separado, con varios frames en vuelo. |
| P2 | Fractura: `SceneManager::apply_fracture`, `RigidBodyManager::carve_and_label` | Reutilizar staging y agrupar trabajo de varios eventos; después mover censo/bounds/compactación de componentes a GPU. Medir p95/p99 del frame que publica roturas y validar conservación y superficies. |
| P2 | Resolución GPU por colores | Compactar trabajo por color o evaluar una ejecución GPU que reduzca despachos vacíos. Conservar orden/historial en la primera variante. Usar replays completos, no solo número de cuerpos dormidos. |
| P2 | Errores de carga/inicialización | Propagar el fallo de `scene_from_file`: actualmente un archivo inexistente solo escribe un error y retorna void. Revisar `return !initialized` en métodos que deben informar éxito; `simulate()` devuelve true precisamente cuando no está inicializado. Añadir pruebas de entradas inválidas/fallos, sin aceptar una escena parcial como benchmark válido. |
| P2 | Pruebas y entrega | Añadir CI CPU y un gauntlet GPU explícito que compruebe finalización, hashes, penetración, NaNs, overflow y capturas. Comprobar que el último commit probado está realmente incluido en el merge. |
| P2 | Portabilidad de colas | Se imprime el número de colas compute pero se usa COMPUTE_0 sin fallback. Centralizar selección de cola y admitir MAIN cuando no exista una compute dedicada; probar el contrato de sincronización en ambas rutas. |
| P3 | Arquitectura/arranque | Extraer SceneManager de su header, separar definición de escenas, pools, serialización y publicación. Revisar reserva de memoria de tareas y la retención global de callbacks. Retirar recursos STBN si siguen sin tener consumidores reales. |

### Lo que ocurre con los semáforos de Daxa

En el checkout instalado, `lib/Daxa/src/utils/impl_task_graph.cpp:1913` contiene
`TaskGraph::submit([[maybe_unused]] TaskSubmitInfo const & info)`: su cuerpo añade
un submit con queue_bits=0 y no lee `info`. Los campos
`additional_wait_timeline_semaphores` y `additional_signal_timeline_semaphores`
aparecen en el header, pero no se consumen en la implementación.

Por tanto, incrementar `sim_timeline_value` y pasar esos spans no implementa la
publicación explícita que describen los comentarios del proyecto. Esto no prueba
que toda la sincronización actual esté rota: también existen dependencias de
recursos gestionadas por Daxa y esperas de CPU. Sí impide usar ese timeline como
justificación para eliminar dichas esperas. Las pruebas actuales de Vulkan pasan;
no sustituyen una prueba del contrato de publicación con trabajo solapado.

### Fractura y convergencia de componentes

La ruta actual hace partición/labels en GPU, descarga occupancy y labels, hace
censo y remapeo en CPU, vuelve a subir fragmentos, reconstruye derivados en GPU y
lee resultados para corregir masa/inercia/COM. El coste no puede atribuirse solo
al tamaño de una descarga: importa la cadena de dependencias y cuántas veces
se espera. Primero medir cada fase por evento y agrupar eventos compatibles.

El límite fijo de flood fill también merece una prueba contra un BFS de referencia.
El comentario de `carve_and_label` afirma convergencia para cualquier forma con
8 + 2*floor(log2(cells)) iteraciones. Una simulación CPU síncrona del stencil
min-neighbor + pointer-jump sobre un serpentín de 32 filas útiles, ancho 32 y
filas separadoras conserva 17 labels tras el presupuesto de 28 iteraciones,
aunque existe un único componente. Es un contraejemplo al supuesto para ese
orden de actualización; **no es una reproducción del kernel in-place en la GPU**.
Antes de cambiarlo, comparar el kernel real con BFS en laberintos, puentes finos y
particiones Voronoi, incluyendo varias ejecuciones y arquitecturas.

### Compilación, memoria y recursos

La separación actual de shaders y la sincronización de fuentes sin alterar mtimes
son útiles y deben conservarse. `shared.inl` sigue siendo un contrato amplio:
cambiar layouts comunes obliga a recompilar sus consumidores. Separar algoritmos
no elimina esa dependencia legítima.

`TaskManager::create_task_graph` reserva 8 MiB de memoria de tareas en una de sus
sobrecargas, con un comentario de 2600 tareas anterior al batching. La otra usa
el valor por defecto. Medir el uso real por graph y establecer presupuestos por
clase de graph; no bajar el tamaño global sin comprobar capacidad. El registro
estático `retain_task_callback` conserva callbacks hasta terminar el proceso: hoy
los resets reutilizan graphs, pero una futura recreación de managers acumularía
capturas. La vida útil debe pertenecer al graph, con el soporte apropiado de Daxa.

ImageManager carga y sube 64 texturas STBN; el acceso del path tracer está comentado
y el RNG activo usa hashes. Verificar consumidores antes de retirar todo ese
camino, sus bindings y los assets. No se ha medido aquí su coste de arranque.

## Validación de esta rama

- Build Release y cinco targets de CTest.
- F7, 1800 pasos, AVBD y TGS: CSV y checkpoints completos idénticos a los controles de la campaña anterior.
- Fixture de fractura, 900 pasos por solver: CSV y checkpoints completos idénticos a los controles.
- Las cuatro ejecuciones usan Vulkan synchronization validation; la fixture también
  verifica pools. No errores reportados.
- Prueba de UI con Vulkan: F9, L/BVH, reset, movimiento de cámara, acumulación,
  cambio a simulación y tamaño 3840x2160. Captura inspeccionada visualmente.
- Prueba de CPU al minimizar ambos ejecutables y restauración/cierre.

Evidencia: `/root/beat-box/work/project-audit/`, con logs de build/CTest,
`scene-*`, `ui-validation.log`, `bvh-camera.png`, `ui-checks.log` y
`flood_budget_probe.py` (modelo CPU).
Las cifras previas de TGS/AVBD siguen en `TGS_SDF_PERFORMANCE.md` y
`SDF_PERFORMANCE.md`; no se atribuyen nuevos speedups del solver a esta revisión.
