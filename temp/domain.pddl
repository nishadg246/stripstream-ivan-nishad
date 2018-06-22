(define (domain stripstream)
(:requirements :strips :typing :equality :disjunctive-preconditions :derived-predicates :action-costs :derived-predicates :adl)
(:types conf pose block)
(:constants
	conf_0 conf_1 conf_10 conf_11 conf_12 conf_13 conf_14 conf_15 conf_16 conf_17 conf_18 conf_19 conf_2 conf_20 conf_21 conf_3 conf_4 conf_5 conf_6 conf_7 conf_8 conf_9 - conf
	pose_0 pose_1 pose_10 pose_11 pose_12 pose_13 pose_14 pose_15 pose_16 pose_17 pose_18 pose_19 pose_2 pose_20 pose_21 pose_22 pose_3 pose_4 pose_5 pose_6 pose_7 pose_8 pose_9 - pose
	block_0 block_1 block_2 - block)
(:predicates
	(safe ?x0 - block ?x1 - block ?x2 - pose)
	(legalkin ?x0 - pose ?x1 - conf)
	(atpose ?x0 - block ?x1 - pose)
	(atconf ?x0 - conf)
	(collisionfree ?x0 - block ?x1 - pose ?x2 - block ?x3 - pose)
	(holding ?x0 - block)
	(handempty))
(:functions (total-cost))

(:action move
	:parameters (?q1 - conf ?q2 - conf)
	:precondition (atconf ?q1)
	:effect (and (atconf ?q2) (not (atconf ?q1))))

(:action place
	:parameters (?b1 - block ?p1 - pose ?q1 - conf)
	:precondition (and (holding ?b1) (atconf ?q1) (legalkin ?p1 ?q1) (forall (?b2 - block) (or (= ?b1 ?b2) (safe ?b2 ?b1 ?p1))))
	:effect (and (atpose ?b1 ?p1) (handempty) (not (holding ?b1))))

(:action pick
	:parameters (?b1 - block ?p1 - pose ?q1 - conf)
	:precondition (and (atpose ?b1 ?p1) (handempty) (atconf ?q1) (legalkin ?p1 ?q1))
	:effect (and (holding ?b1) (not (atpose ?b1 ?p1)) (not (handempty))))

(:derived (safe ?b2 - block ?b1 - block ?p1 - pose)
	(exists (?p2 - pose) (and (atpose ?b2 ?p2) (collisionfree ?b1 ?p1 ?b2 ?p2)))))
