package frc.robot.sim.simMechanism;

import frc.robot.sim.simField.SimGamePiece;
import frc.robot.sim.simField.SimGamePiece.GamePieceVariant;
import java.util.LinkedList;
import java.util.Optional;
import java.util.Queue;
import java.util.Set;

/** Represents an indexer in the sim environment. */
public class SimIndexer {
  private final Queue<SimGamePiece> queue;
  private final Set<GamePieceVariant> allowed;
  private final int capacity;

  public SimIndexer(int capacity, GamePieceVariant... variants) {
    this.queue = new LinkedList<>();
    this.capacity = capacity;
    this.allowed = Set.of(variants);
  }

  /** Forcefully insert a piece (used by intake on successful grab). */
  boolean forceInsertGamePiece(SimGamePiece piece) {
    if (queue.size() < capacity && (allowed.isEmpty() || allowed.contains(piece.variant()))) {
      // piece.intake() moves it into HELD
      piece.intake();
      queue.add(piece);
      return true;
    }
    return false;
  }

  /** Inserts a game piece if it’s user-controlled and allowed. */
  public boolean insertGamePiece(SimGamePiece piece) {
    if (!piece.isUserControlled()) {
      return false;
    }
    return forceInsertGamePiece(piece);
  }

  /** Removes and returns the next piece, or empty if none. */
  public Optional<SimGamePiece> removeGamePiece() {
    SimGamePiece p = queue.poll();
    return p != null ? Optional.of(p.userControlled()) : Optional.empty();
  }

  /** Clears all stored pieces (sending them to LIMBO). */
  public void clear() {
    for (var p : queue) {
      p.delete();
    }
    queue.clear();
  }
}
