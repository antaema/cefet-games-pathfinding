package br.cefetmg.games;

import br.cefetmg.games.graphics.Facing;
import br.cefetmg.games.movement.Position;
import br.cefetmg.games.movement.Steering;
import br.cefetmg.games.movement.Target;
import br.cefetmg.games.movement.behavior.Algorithm;
import br.cefetmg.games.movement.behavior.Seek;
import br.cefetmg.games.pathfinding.TileConnection;
import br.cefetmg.games.pathfinding.TileNode;
import com.badlogic.gdx.ai.pfa.Connection;
import com.badlogic.gdx.ai.pfa.DefaultGraphPath;
import com.badlogic.gdx.ai.pfa.Heuristic;
import com.badlogic.gdx.ai.pfa.indexed.IndexedAStarPathFinder;
import com.badlogic.gdx.ai.pfa.indexed.IndexedAStarPathFinder.Metrics;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Array;
import java.util.Arrays;
import java.util.Iterator;

/**
 *
 * @author Flávio Coutinho <fegemo@gmail.com>
 */
public class Agent {

    public Position position;
    private final Algorithm seek;
    private final IndexedAStarPathFinder pathFinder;
    private final DefaultGraphPath<TileConnection> path;
    private Iterator<TileConnection> pathIterator;
    private final Target steeringTarget;
    private final float fullSpeed = 75;
    private static final float MIN_DISTANCE_CONSIDERED_ZERO_SQUARED = (float) Math.pow(2.0f, 2);
    private Facing facing;
    private TileNode nextNode, currentNode;

    public Color color;
    private boolean shouldMove;

    public Agent(Vector2 position, Color color) {
        this.position = new Position(position);
        this.color = color;
        this.steeringTarget = new Target(position);
        this.seek = new Seek(fullSpeed);
        this.seek.target = steeringTarget;
        this.pathFinder = new IndexedAStarPathFinder(LevelManager.graph, true);
        this.path = new DefaultGraphPath<>();
        this.pathIterator = this.path.iterator();
        this.facing = Facing.EAST;
        this.shouldMove = false;
    }

    /**
     * Atualiza a posição do agente de acordo com seu objetivo de alto nível
     * (pathfinding).
     *
     * @param delta tempo desde a última atualização.
     */
    public void update(float delta) {
        shouldMove = true;

        // verifica se atingimos nosso objetivo imediato
        if (position.coords.dst2(steeringTarget.coords) < MIN_DISTANCE_CONSIDERED_ZERO_SQUARED) {
            // procurar se temos outra conexão na nossa rota
            // e, caso afirmativo, definir o nó de chegada como novo target
            if (shouldMove = pathIterator.hasNext()) {
                TileConnection nextConnection = pathIterator.next();
                nextNode = nextConnection.getToNode();
                steeringTarget.coords = nextNode.getPosition();

                // atualiza a velocidade do "seek" de acordo com o terreno (a conexão)
                this.seek.maxSpeed = fullSpeed - (fullSpeed / 2.0f) * (nextConnection.getCost() - 1) / (LevelManager.maxCost - 1);
            }
        } else if (position.coords.dst2(steeringTarget.coords) < MIN_DISTANCE_CONSIDERED_ZERO_SQUARED * 6) {
            currentNode = nextNode;
        }

        // integra
        if (shouldMove) {
            Steering steering = seek.steer(this.position);
            position.integrate(steering, delta);

            // verifica o vetor velocidade para determinar a orientação
            float angle = steering.velocity.angle();
            int quadrant = (int) (((int) angle + (360 - 67.5f)) / 45) % 8;
            facing = Facing.values()[(8 - quadrant) % 8];
        }
    }

    public float getCost(TileNode n, TileNode n1) {
        Array<Connection<TileNode>> connections = n.getConnections();
        for (Connection<TileNode> t : connections) {
            if (t instanceof TileConnection) {
                t = (TileConnection) t;
                if (t.getToNode().isObstacle() == false) {
                    if (n == t.getFromNode() && n1 == t.getToNode()) {
                        return t.getCost();
                    }
                }
            }
        }
        return (0);
    }

    /**
     * Este método é chamado quando um clique no mapa é realizado.
     *
     * @param x coordenada x do ponteiro do mouse.
     * @param y coordenada y do ponteiro do mouse.
     */
    public void setGoal(int x, int y) {
        TileNode startNode = LevelManager.graph
                .getNodeAtCoordinates(
                        (int) this.position.coords.x,
                        (int) this.position.coords.y);
        TileNode targetNode = LevelManager.graph
                .getNodeAtCoordinates(x, y);

        path.clear();
        pathFinder.metrics.reset();
        // AQUI ESTAMOS CHAMANDO O ALGORITMO A* (instância pathFinder) 
        pathFinder.searchConnectionPath(startNode, targetNode,
                new Heuristic<TileNode>() {
            @Override
            public float estimate(TileNode n, TileNode n1) {
                /*
                throw new UnsupportedOperationException("Deveria ter retornado "
                        + "um valor para a heurística no arquivo "
                        + "Agent.java:107, mas o professor resolveu explodir "
                        + "o programa e deixar você consertar ;)"); 
                 */
                float cost = getCost(n,n1);
                //Dijkstra 
                //  return n.getPosition().dst(n1.getPosition())/LevelManager.tileHeight;
                /*
                // Heuristica 1
                float porcentage = 0.5f;
                if ( porcentage * cost > 1) {
                    return n.getPosition().dst(n1.getPosition()) * porcentage * cost / LevelManager.tileHeight;
                } else {
                    return n.getPosition().dst(n1.getPosition()) / LevelManager.tileHeight;
                }
                */
                // Heuristica 2
                float weighting = 1;
                float crossweighting = (float) Math.sqrt(2);
                float deltaX = Math.abs(n.getPosition().x-n1.getPosition().x);
                float deltaY = Math.abs(n.getPosition().y-n1.getPosition().y);
                return (weighting * (deltaX+deltaY) + (crossweighting - 2 * weighting) * Math.min(deltaX, deltaY)) / (LevelManager.tileWidth);
            }
        }, path);
        pathIterator = path.iterator();
    }

    /**
     * Retorna em que direção (das 8) o agente está olhando.
     *
     * @return a direção de orientação.
     */
    public Facing getFacing() {
        return facing;
    }

    /**
     * Retorna se o agente está se movimentando ou se está parado.
     *
     * @return
     */
    public boolean isMoving() {
        return shouldMove;
    }

    /**
     * Retorna se o agente está em um tile de água.
     *
     * @return
     */
    public boolean isUnderWater() {
        return currentNode == null ? false : currentNode.isWater();
    }

    /**
     * Retorna as métricas da última execução do algoritmo de planejamento de
     * trajetórias.
     *
     * @return as métricas.
     */
    public Metrics getPathFindingMetrics() {
        return pathFinder.metrics;
    }
}
