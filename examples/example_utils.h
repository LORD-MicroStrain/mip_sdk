#include <mip/definitions/data_filter.h>


// Displays current filter state for the connected device if it has changed.
void displayFilterState(
    const mip_filter_mode filter_status,
    char **current_state,
    bool isFiveSeries
);
