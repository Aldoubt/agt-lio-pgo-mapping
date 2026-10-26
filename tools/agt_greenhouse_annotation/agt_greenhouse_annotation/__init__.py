"""Offline manual greenhouse topology and automatic structure evidence."""

__version__ = '0.2.0'

from .navigation_map_derivation import (
    FREE, OCCUPIED, UNKNOWN, GroundRelativeNavigationConfig, NavigationMapResult,
    derive_ground_relative_navigation_map,
)
from .navigation_structure import (
    NavigationStructureConfig, NavigationStructureResult, RowModel,
    derive_navigation_structure,
)
from .terrain_morphology import (
    TerrainMorphologyConfig, TerrainMorphologyResult, derive_terrain_morphology,
)
from .navigation_row_tracks import (
    LocalRowObservation, LocalRowTrackingConfig, RowTrack, LocalRowTrackResult,
    derive_local_row_tracks,
)
from .navigation_corridor import (
    AislePairDiagnostic, CorridorRefinementConfig, CorridorRefinementResult,
    derive_corridor_refinement,
)
from .aisle_centerlines import (
    GeometricAisleCenterlineResult, GeometricAislePair, derive_geometric_aisle_centerlines,
)
from .structure_artifacts import (
    AisleProposal, GlobalRowProposal, GreenhouseStructureAnalysis,
    GreenhouseStructureConfig, GreenhouseTerrainEvidence,
    analyze_greenhouse_structure, freeze_accepted_proposals,
    load_proposal_artifact, save_proposal_revision, save_review_revision,
    make_boundary_aisle_proposal, proposal_aisles_for_rows,
    decision_for_proposal, merge_row_proposals, split_row_proposal,
)
from .agricultural_aisle_graph import (
    AisleGraphConfig, AislePrimitive, AgriculturalAisleGraph,
    derive_agricultural_aisle_graph, write_agricultural_aisle_graph,
)

from .review_3d import deterministic_display_sample
