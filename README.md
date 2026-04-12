# CE 4011 Assignment 3 — 2D Structural Analysis Program

**Ali Utku Tekin — 2744076**  
MSc. Structural Engineering, 3rd Semester | Spring 2025-2026

## Overview

Extended 2D structural analysis program using the Direct Stiffness Method with NumPy.

## Features

| Feature | Implementation |
|---|---|
| Frame elements | `FrameElement2D` — 6-DOF beam-column (inheritance from `Element2D`) |
| Truss elements | `TrussElement2D` — axial-only, rotational DOFs auto-suppressed |
| Moment releases | Schur-complement static condensation (exact for k and p) |
| Member loads | UDL and point loads via Hermitian shape functions |
| Mechanism detection | SVD rank check with null-space DOF identification |
| Connectivity check | DFS graph traversal — detects floating components |

## Architecture
```mermaid
classDiagram
    direction TB

    class Node {
        <<frozen>>
        +int id
        +float x
        +float y
    }

    class Material {
        <<frozen>>
        +int id
        +float E
        +float A
        +float I
    }

    class Support {
        <<frozen>>
        +int node_id
        +bool ux
        +bool uy
        +bool rz
    }

    class NodalLoad {
        <<frozen>>
        +int node_id
        +float fx
        +float fy
        +float mz
    }

    class UniformDistributedLoad {
        <<frozen>>
        +float wy
    }

    class PointLoad {
        <<frozen>>
        +float py
        +float a
    }

    class StructuralModel {
        +str title
        +dict~int,Node~ nodes
        +dict~int,Material~ materials
        +list~Element2D~ elements
        +dict~int,Support~ supports
        +list~NodalLoad~ nodal_loads
        +node(node_id) Node
        +support_for(node_id) Support
        +node_ids() list~int~
    }

    class AnalysisResult {
        +str status
        +str title
        +list~str~ warnings
        +dict E_map
        +int num_eq
        +ndarray K
        +ndarray F
        +ndarray D
        +float residual
        +dict member_results
        +dict reactions
    }

    class Element2D {
        <<abstract>>
        +int id
        +int node_i
        +int node_j
        +float E
        +float A
        +list~MemberLoad~ member_loads
        +kind()* str
        +raw_local_stiffness(nodes)* ndarray
        +length_cos_sin(nodes) tuple
        +transformation_matrix(nodes) ndarray
        +local_consistent_load(nodes) ndarray
        +assembly_local_indices() list
        +assembled_local_stiffness_and_load(nodes) tuple
        +global_stiffness_and_load(nodes) tuple
        +local_displacement_and_end_forces(nodes, u) tuple
    }

    class FrameElement2D {
        +float I
        +bool release_i
        +bool release_j
        +kind() str
        +raw_local_stiffness(nodes) ndarray
        +local_consistent_load(nodes) ndarray
        +assembled_local_stiffness_and_load(nodes) tuple
        +local_displacement_and_end_forces(nodes, u) tuple
        -_released_dofs() list~int~
    }

    class TrussElement2D {
        +kind() str
        +raw_local_stiffness(nodes) ndarray
        +assembly_local_indices() list
    }

    class DofManager {
        +dict active_map
        +list~int~ free_indices
        +list~int~ restrained_indices
        +dict~int,str~ labels
        +int n_total
        +from_model(model)$ DofManager
        +index(node_id, dof) int|None
        +element_dof_map(elem) list
        +e_matrix_for_display(model) dict
        +g_vector_for_display(elem) list
    }

    Element2D <|-- FrameElement2D
    Element2D <|-- TrussElement2D
    FrameElement2D *-- UniformDistributedLoad : member_loads
    FrameElement2D *-- PointLoad : member_loads
    StructuralModel *-- Node
    StructuralModel *-- Material
    StructuralModel *-- Element2D
    StructuralModel *-- Support
    StructuralModel *-- NodalLoad

    note for Element2D "Schur complement condensation\nfor moment releases"
    note for DofManager "Auto-omits Rz at\npure-truss nodes"
```
```
Element2D (abstract base)
├── FrameElement2D  — axial + flexural, optional releases
└── TrussElement2D  — axial only, suppresses Rz DOFs

DofManager — dynamic equation numbering, auto-omits Rz at pure-truss nodes
```

## Usage

```bash
python -m structural_analysis.main inputs/portal_frame.txt
python -m pytest tests/test_all.py -v   # 45 tests
```

## Test Suite (45 tests)

### Q3(c) Updated Test Case

Run:
python run_q3c_vertical_frame_case.py

This case demonstrates:
- disconnected but supported substructures
- block-diagonal stiffness behavior
- warning + successful solution

- **22 unit tests**: geometry, frame/truss stiffness, releases, FEF
- **9 interface tests**: DOF numbering, assembly, validation
- **14 regression tests**: portal frame (A2), cantilever, SS beam, truss, mechanism, hinge+load

## AI Reference

Claude (Anthropic) and ChatGPT (OpenAI).
