"""Centralized publication-quality plotting configuration.

All figures across the simulation experiment pipeline should import and call
`setup_publication_style()` before generating plots. This ensures a consistent,
paper-ready appearance (font sizes, color palette, grid style, output formats).
"""
import os
import matplotlib
import matplotlib.pyplot as plt

try:
    import seaborn as sns
    SEABORN_AVAILABLE = True
except ImportError:
    SEABORN_AVAILABLE = False

# ---------------------------------------------------------------------------
# Default style parameters (override via experiment_config.json -> plotting)
# ---------------------------------------------------------------------------
DEFAULT_FONT_BASE_SIZE_POINTS = 14
DEFAULT_FONT_AXIS_LABEL_SIZE_POINTS = 16
DEFAULT_FONT_TITLE_SIZE_POINTS = 18
DEFAULT_OUTPUT_RESOLUTION_DPI = 300
DEFAULT_COLOR_PALETTE_NAME = "colorblind"
DEFAULT_GRID_LINE_ALPHA = 0.5
DEFAULT_ENABLE_STATISTICAL_ANNOTATION = False
DEFAULT_OUTPUT_FORMAT_LIST = ["png", "pdf"]

# Figure size maps
FIGSIZE_SINGLE = (8, 6)      # e.g., one box plot, one line chart
FIGSIZE_GROUPED = (12, 6)    # e.g., grouped bar chart with many schedulers


def setup_publication_style(
    font_base_size_points: int = DEFAULT_FONT_BASE_SIZE_POINTS,
    font_axis_label_size_points: int = DEFAULT_FONT_AXIS_LABEL_SIZE_POINTS,
    font_title_size_points: int = DEFAULT_FONT_TITLE_SIZE_POINTS,
    color_palette_name: str = DEFAULT_COLOR_PALETTE_NAME,
    grid_line_alpha: float = DEFAULT_GRID_LINE_ALPHA,
):
    """Configure matplotlib rcParams for publication-quality figures.

    Parameters
    ----------
    font_base_size_points
        Base font size for all text elements.
    font_axis_label_size_points
        Font size for x-axis and y-axis labels.
    font_title_size_points
        Font size for figure titles.
    color_palette_name
        Name of the color palette. Supported values depend on whether seaborn
        is installed: "colorblind", "deep", "muted", "bright", "dark",
        "Set2", "tab10", etc.
    grid_line_alpha
        Transparency of grid lines (0 = invisible, 1 = fully opaque).
    """
    plt.rcParams.update({
        "font.size": font_base_size_points,
        "axes.labelsize": font_axis_label_size_points,
        "axes.titlesize": font_title_size_points,
        "xtick.labelsize": font_base_size_points,
        "ytick.labelsize": font_base_size_points,
        "legend.fontsize": font_base_size_points - 1,
        "figure.dpi": DEFAULT_OUTPUT_RESOLUTION_DPI,
        "savefig.dpi": DEFAULT_OUTPUT_RESOLUTION_DPI,
        "axes.grid": True,
        "grid.alpha": grid_line_alpha,
        "grid.linestyle": "--",
        "axes.axisbelow": True,
    })

    # Apply color palette if seaborn is available
    if SEABORN_AVAILABLE:
        try:
            sns.set_palette(color_palette_name)
        except ValueError:
            # Fallback if seaborn doesn't recognise the name
            pass


def get_scheduler_color_map(scheduler_list, palette_name=DEFAULT_COLOR_PALETTE_NAME):
    """Return a dict mapping scheduler name to a consistent RGB color.

    Parameters
    ----------
    scheduler_list : list[str]
        Ordered list of scheduler names.
    palette_name : str
        Palette name to draw from.

    Returns
    -------
    dict[str, tuple]
        Mapping scheduler -> (R, G, B) float tuple.
    """
    if SEABORN_AVAILABLE:
        palette = sns.color_palette(palette_name, n_colors=len(scheduler_list))
    else:
        # Fallback to matplotlib tab10 if seaborn absent
        cmap = matplotlib.colormaps.get_cmap("tab10")
        palette = [cmap(i) for i in range(len(scheduler_list))]
    return {s: palette[i] for i, s in enumerate(scheduler_list)}


def save_figure(fig, output_path_stem, format_list=None, dpi=None):
    """Save a matplotlib figure in multiple formats.

    Parameters
    ----------
    fig : matplotlib.figure.Figure
        The figure to save.
    output_path_stem : str
        Output path without extension (e.g., ``figures/fig1a_mean_sp``).
    format_list : list[str] | None
        List of extensions such as ``["png", "pdf"]``. Defaults to
        ``DEFAULT_OUTPUT_FORMAT_LIST``.
    dpi : int | None
        Resolution; defaults to ``DEFAULT_OUTPUT_RESOLUTION_DPI``.
    """
    if format_list is None:
        format_list = DEFAULT_OUTPUT_FORMAT_LIST
    if dpi is None:
        dpi = DEFAULT_OUTPUT_RESOLUTION_DPI

    output_dir = os.path.dirname(output_path_stem)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    for fmt in format_list:
        fname = f"{output_path_stem}.{fmt}"
        fig.savefig(fname, format=fmt, dpi=dpi, bbox_inches="tight")
        print(f"Saved figure: {fname}")
