import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os

# --- CONFIGURATION ---
# Note: Ensure these CSV files are in your active directory.
file_lazy = "lazy_system.csv"
file_crit = "critically_damped_system.csv"
target_rpm = 400
plot_title = "Motor Step Response: P vs. PI Controller Configuration"

# Colors (Texas A&M Theme + Professional Slate)
AGGIE_MAROON = "#500000"
SLATE_BLUE = "#2C3E50"
STEEL_GRAY = "#7F8C8D"  
# ---------------------

def prepare_data(file_name):
    """Loads CSV and zeros the time relative to the moment the motor starts spinning."""
    if not os.path.exists(file_name):
        print(f"Error: Could not find {file_name}")
        return None
        
    df = pd.read_csv(file_name)
    
    # Filter to start exactly when the motor begins moving (RPM > 0)
    df_active = df[df['RPM'] > 0].copy()
    if df_active.empty:
        print(f"Error: No movement found in {file_name}")
        return None
        
    # Normalize timestamp so t=0 is the exact start of movement
    start_time = df_active['Timestamp(ms)'].iloc[0]
    df_active['Time_s'] = (df_active['Timestamp(ms)'] - start_time) / 1000.0
    
    return df_active

def main():
    lazy_df = prepare_data(file_lazy)
    crit_df = prepare_data(file_crit)
    
    if lazy_df is None or crit_df is None:
        return

    # Apply modern styling - context="poster" scales UI elements up natively
    sns.set_theme(style="whitegrid", context="poster")
    plt.rcParams['font.family'] = 'sans-serif'
    plt.rcParams['axes.edgecolor'] = '#D3D3D3'

    # Create figure with high resolution and large dimensions for a poster
    fig, ax = plt.subplots(figsize=(16, 10), dpi=300)
    
    # 1. Draw the shaded "Settling Band" (± 2% of Target RPM) anchored at 400 RPM
    margin = target_rpm * 0.02
    ax.fill_between([0, 10], target_rpm - margin, target_rpm + margin, 
                    color=SLATE_BLUE, alpha=0.15, label='±2% Settling Band', edgecolor='none')

    # 2. Draw the Target Line (anchored rigidly at 400 RPM)
    ax.axhline(y=target_rpm, color=SLATE_BLUE, linestyle='--', linewidth=4.0, label='Target Speed')

    # 3. Draw the data lines for BOTH systems (Made exceptionally thick for poster visibility)
    # Critically Damped System -> Underdamped (PI Controller)
    ax.plot(crit_df['Time_s'], crit_df['RPM'], color=AGGIE_MAROON, linewidth=4.5, 
            label='Aggressive PI Controller (Kp + Ki)')
    
    # Lazy System -> Underdamped (P Controller)
    ax.plot(lazy_df['Time_s'], lazy_df['RPM'], color=STEEL_GRAY, linewidth=4.5, 
            label='Conservative P Controller (Kp only)')

    # Styling and Labels - Poster Scaled Fonts
    plt.title(plot_title, fontsize=32, fontweight='bold', pad=35, color='#1A1A1A', loc='center')
    plt.xlabel('Time (Seconds)', fontsize=26, fontweight='bold', labelpad=20)
    plt.ylabel('Speed (RPM)', fontsize=26, fontweight='bold', labelpad=20)
    
    # Increase tick label size for readability from a distance
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)

    # Clean up axes: X bounded exactly 0 to 10 seconds. Y bounded slightly below 0.
    plt.ylim(-20, target_rpm + 200)
    plt.xlim(0, 10.0)    
    
    # Make grid lines softer but visible
    ax.grid(color='#E0E0E0', linestyle='-', linewidth=1.5)
    sns.despine(left=True, bottom=True)

    # --- TEXT BOXES FOR PRESENTATION CONTEXT ---
    
    # Target Speed Callout (Top Right)
    text_target = f'Target Speed: {target_rpm} RPM'
    props_target = dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.95, edgecolor=SLATE_BLUE, linewidth=2)
    ax.text(0.98, 0.95, text_target, transform=ax.transAxes, color=SLATE_BLUE, fontsize=22,
            fontweight='bold', ha='right', va='top', bbox=props_target)

    # Argument for Mobile App Tuning Flexibility (Bottom Left)
    textstr = '\n'.join((
        r'$\bf{App-Based\ Tuning\ Advantage:}$',
        '• Adjust Kp & Ki dynamically via mobile app.',
        '• High Ki (Maroon): Best for fast response & heavy loads.',
        '• Low/Zero Ki (Gray): Best for gentle starts & fragile loads.',
        '• Eliminates the need for hardcoded MCU constraints.'
    ))
    props_params = dict(boxstyle='round,pad=0.6', facecolor='#F8F9FA', alpha=0.95, edgecolor='#A6ACAF', linewidth=2)
    ax.text(0.02, 0.04, textstr, transform=ax.transAxes, fontsize=18,
            verticalalignment='bottom', bbox=props_params, color='#2C3E50')

    # --- LEGEND PLACEMENT ---
    # Forced strictly to the lower right corner, avoiding the text box on the left and the data spikes
    legend = plt.legend(loc='lower right', frameon=True, shadow=True, fancybox=True, fontsize=18)
    legend.get_frame().set_facecolor('white')
    legend.get_frame().set_edgecolor('#A6ACAF')
    legend.get_frame().set_linewidth(2)

    # Save and output
    output_image = "poster_p_vs_pi_comparison.png"
    plt.tight_layout()
    plt.savefig(output_image, bbox_inches='tight')
    print(f"Poster graph successfully saved as {output_image}")

if __name__ == "__main__":
    main()