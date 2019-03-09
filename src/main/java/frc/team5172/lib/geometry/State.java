package frc.team5172.lib.geometry;

import frc.team5172.lib.util.CSVWritable;
import frc.team5172.lib.util.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
