// -*-c++-*-

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3, or (at your option)
 any later version.

 This code is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this code; see the file COPYING.  If not, write to
 the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.

 *EndCopyright:
 */

// Student Soccer 2D Simulation Base , STDAGENT2D
// Simplified the Agent2D Base for HighSchool Students.
// Technical Committee of Soccer 2D Simulation League, IranOpen
// Nader Zare
// Mostafa Sayahi
// Pooria Kaviani
/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "bhv_basic_offensive_kick.h"

#include "basic_actions/body_hold_ball.h"
#include "basic_actions/body_smart_kick.h"
#include "basic_actions/body_stop_ball.h"
#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>

#include <rcsc/geom/sector_2d.h>

#include <vector>
using namespace rcsc;

/*-------------------------------------------------------------------*/
/*!

 */

bool IsSafeArea(const WorldModel &wm, const Sector2D area, const AbstractPlayerObject * player){
    if(area.contains(player->pos())){
        return false;
    }

    return true;
}

int CountPlayerInArea(const WorldModel &wm, const Sector2D &area) 
{
    Vector2D ball_pos = wm.ball().pos();
    int counter = 0;
    for (int u = 1; u <= 11; u++)
    {
        const AbstractPlayerObject *tm = wm.theirPlayer(u);
        if (tm == NULL || tm->unum() < 1 )
            continue;
        Vector2D tm_pos = tm->pos();
        if (tm->pos().dist(ball_pos) > 30)
            continue;
        if (!IsSafeArea(wm, area, tm))
        {
            counter++;
        }
    }
    return counter;
}

double pointToSegment(const WorldModel &wm, Vector2D& player, const AbstractPlayerObject *opponent){
    Vector2D ball_pos = wm.ball().pos();
    Vector2D opp_pos = opponent->pos();

    // Calcula a distância do oponente à linha que liga a bola ao jogador
    double segment_equation = std::fabs((player.y - ball_pos.y) * opp_pos.x
                               - (player.x - ball_pos.x) * opp_pos.y
                               + player.x * ball_pos.y
                               - player.y * ball_pos.x);

    double distance = std::sqrt(std::pow(player.y - ball_pos.y, 2)
                                 + std::pow(player.x - ball_pos.x, 2));

    if (distance == 0.0) return 0.0; // evita divisão por zero

    return segment_equation/ distance;
    
}

double NearestOpponentToLine(const WorldModel &wm, Vector2D& player){

    Vector2D ball_pos = wm.ball().pos();
    int nearest_dist_opponent = 30; 
    for (int u = 1; u<=11; u++){
        const AbstractPlayerObject *opponent = wm.theirPlayer(u);
        if (opponent == NULL || opponent->unum() < 1)
            continue;
        Vector2D opp_pos = opponent->pos();
        if (opp_pos.dist(ball_pos) > 30)
            continue;
        if ( pointToSegment(wm, player, opponent) < nearest_dist_opponent)
        {
            nearest_dist_opponent = pointToSegment(wm, player, opponent);
        }
    }
    return nearest_dist_opponent;

}



bool Bhv_BasicOffensiveKick::execute(PlayerAgent *agent)
{
    dlog.addText(Logger::TEAM,
                 __FILE__ ": Bhv_BasicOffensiveKick");

    const WorldModel &wm = agent->world();

    if (shoot(agent))
    {
        return true;
    }

    const auto &opps = wm.opponentsFromSelf();
    const PlayerObject *nearest_opp = (opps.empty()
                                           ? static_cast<PlayerObject *>(0)
                                           : opps.front());
    const double nearest_opp_dist = (nearest_opp
                                         ? nearest_opp->distFromSelf()
                                         : 1000.0);
    //    const Vector2D nearest_opp_pos = ( nearest_opp
    //                                       ? nearest_opp->pos()
    //                                       : Vector2D( -1000.0, 0.0 ) );

    if (nearest_opp_dist < 10)
    {
        if (pass(agent))
            return true;
    }

    if (dribble(agent))
    {
        return true;
    }

    if (nearest_opp_dist > 2.5)
    {
        dlog.addText(Logger::TEAM,
                     __FILE__ ": hold");
        agent->debugClient().addMessage("OffKickHold");
        Body_HoldBall().execute(agent);
        return true;
    }
    clearball(agent);
    return true;
}

bool Bhv_BasicOffensiveKick::shoot(rcsc::PlayerAgent *agent)
{
    const WorldModel &wm = agent->world();
    Vector2D ball_pos = wm.ball().pos();
    Vector2D center_goal = Vector2D(52.5, 0);
    if (ball_pos.dist(center_goal) > 25)
        return false;
    Vector2D left_goal = Vector2D(52.5, 6);
    Vector2D right_goal = Vector2D(52.5, -6);

    if (left_goal.dist(ball_pos) < right_goal.dist(ball_pos))
    {
        Body_SmartKick(left_goal, 3, 0.1, 2).execute(agent);
    }
    else
    {
        Body_SmartKick(right_goal, 3, 0.1, 2).execute(agent);
    }
    return true;
}

bool Bhv_BasicOffensiveKick::pass(PlayerAgent *agent, int kick_count)
{
    const WorldModel &wm = agent->world();
    std::vector<Vector2D> targets;
    Vector2D ball_pos = wm.ball().pos();
    const ServerParam & SP = ServerParam::i();
    
    //double speed_max =SP.defaultPlayerSpeedMax();

    for (int u = 1; u <= 11; u++)
    {
        const AbstractPlayerObject *tm = wm.ourPlayer(u);
        if (tm == NULL || tm->unum() < 1 || tm->unum() == wm.self().unum())
            continue;
        Vector2D tm_pos = tm->pos();
        if (tm->pos().dist(ball_pos) > 30)
            continue;
        Sector2D pass = Sector2D(ball_pos, 1, tm_pos.dist(ball_pos) + 3, (tm_pos - ball_pos).th() - 15, (tm_pos - ball_pos).th() + 15);
        if (!wm.existOpponentIn(pass, 5, true))
        {
            targets.push_back(tm_pos);
        }
    }
   
const AbstractPlayerObject *best_teammate = NULL;
Vector2D best_option;
double best_safety_score = -1; 

for (auto target : targets) {
    const AbstractPlayerObject *teammate = NULL;

    // Encontrar o companheiro de equipe mais próximo do alvo
    for (int u = 1; u <= 11; u++) {
        const AbstractPlayerObject *tm = wm.ourPlayer(u);
        if (tm != NULL && tm->pos().dist(target) < 1.5) {  
            teammate = tm;
            break;
        }
    }
    
    if (teammate == NULL) continue;  

    // Criar setor em torno do alvo para avaliar segurança
    Sector2D target_sector(target, 1, 5, 0, 360); 
    int opponents_near_target = CountPlayerInArea(wm, target_sector);

    Vector2D self_to_target = target - wm.self().pos();
    double nearest_opponent_dist = NearestOpponentToLine(wm, target);
    int opponents_in_path = 0;

    for (int u = 1; u <= 11; u++) {
        const AbstractPlayerObject *opponent = wm.theirPlayer(u);
        if (opponent == NULL) continue;

        double dist_opponent_segment = pointToSegment(wm, self_to_target, opponent);

        // Se o oponente estiver muito perto da linha do passe, aumenta a contagem
        if (dist_opponent_segment < 3.0) {  
            opponents_in_path++;
        }
    }

    //double pass_distance = wm.self().pos().dist(target);
    //double pass_speed_bonus = (pass_distance < 15.0) ? 1.5 : 0.0;

    double safety_score = nearest_opponent_dist - (opponents_in_path * 1.5) - (opponents_near_target * 2.0); //+ pass_speed_bonus;

    if (safety_score > best_safety_score) {
        best_safety_score = safety_score;
        best_option = target;
        best_teammate = teammate;
    }
}

if (best_teammate == NULL) {
    return false;
}

    if (targets.size() == 0)
        return false;
        //mVector2D best_target = targets[0];
    
 /*     for (unsigned int i = 1; i < targets.size(); i++)
    {
        if (targets[i].x > best_target.x)
            best_target = targets[i];
    }
            */
    if (wm.gameMode().type() != GameMode::PlayOn)
        Body_SmartKick(best_option, kick_count, 2.5, 1).execute(agent);
    else
        Body_SmartKick(best_option, kick_count, 2.5, 2).execute(agent);
    return true;
}
bool Bhv_BasicOffensiveKick::dribble(PlayerAgent *agent)
{
    const WorldModel &wm = agent->world();
    Vector2D ball_pos = wm.ball().pos();
    double dribble_angle = (Vector2D(52.5, 0) - ball_pos).th().degree();
    Sector2D dribble_sector = Sector2D(ball_pos, 0, 3, dribble_angle - 15, dribble_angle + 15);
    if (!wm.existOpponentIn(dribble_sector, 5, true))
    {
        Vector2D target = Vector2D::polar2vector(3, dribble_angle) + ball_pos;
        if (Body_SmartKick(target, 0.8, 0.7, 2).execute(agent))
        {
            return true;
        }
    }
    return false;
}

bool Bhv_BasicOffensiveKick::clearball(PlayerAgent *agent)
{
    const WorldModel &wm = agent->world();
    if (!wm.self().isKickable())
        return false;
    Vector2D ball_pos = wm.ball().pos();
    Vector2D target = Vector2D(52.5, 0);
    if (ball_pos.x < 0)
    {
        if (ball_pos.x > -25)
        {
            if (ball_pos.dist(Vector2D(0, -34)) < ball_pos.dist(Vector2D(0, +34)))
            {
                target = Vector2D(0, -34);
            }
            else
            {
                target = Vector2D(0, +34);
            }
        }
        else
        {
            if (ball_pos.absY() < 10 && ball_pos.x < -10)
            {
                if (ball_pos.y > 0)
                {
                    target = Vector2D(-52, 20);
                }
                else
                {
                    target = Vector2D(-52, -20);
                }
            }
            else
            {
                if (ball_pos.y > 0)
                {
                    target = Vector2D(ball_pos.x, 34);
                }
                else
                {
                    target = Vector2D(ball_pos.x, -34);
                }
            }
        }
    }
    if (Body_SmartKick(target, 2.7, 2.7, 2).execute(agent))
    {
        return true;
    }
    Body_StopBall().execute(agent);
    return true;
}