#! /usr/bin/env python
# -*- coding: utf-8 -*-
def speech_rec_correction(rec_result):
    #corrction for locations
    if 'planet' in rec_result:
        rec_result[rec_result.index('planet')] = 'toilet'
    if 'tv' in rec_result:
        rec_result[rec_result.index('tv')] = 'tvtable'
    if 'couch' in rec_result:
        rec_result[rec_result.index('couch')] = 'TV couch'
    if 'coach' in rec_result:
        rec_result[rec_result.index('coach')] = 'TV couch'
    if 'bathing' in rec_result:
        rec_result[rec_result.index('bathing')] = 'washbasin'
    if 'machine' in rec_result:
        rec_result[rec_result.index('machine')] = 'washing machine'
    if 'arm' in rec_result and 'chair' in rec_result:
        if rec_result.index('arm') + 1 == rec_result.index('chair'):
            rec_result[rec_result.index('arm')]='armfchair'
            rec_result.pop(rec_result.index('chair'))
    if 'colorado' in rec_result:
        rec_result[rec_result.index('colorado')] = 'corridor'
    if 'coffee' in rec_result:
        rec_result[rec_result.index('coffee')] = 'coffee table'
    if 'tower' in rec_result:
        rec_result[rec_result.index('tower')] = 'towel rail'
    if 'towel' in rec_result:
        rec_result[rec_result.index('towel')] = 'towel rail'
    if 'living' in rec_result:
        rec_result[rec_result.index('living')] = 'livingroom'
    if 'living' in rec_result and 'table' in rec_result:
        rec_result[rec_result.index('living')] = 'livingroom'
    if 'dining' in rec_result:
        rec_result[rec_result.index('dining')] = 'dining_room'
    if 'tiny' in rec_result:
        rec_result[rec_result.index('tiny')] = 'dining_room'

#correction for obj
    if 'tuna' in rec_result:
        rec_result[rec_result.index('tuna')] = 'tuna fish'
    if 'M' in rec_result:
        rec_result[rec_result.index('M')] = 'M and M\'s'
    if 'pair' in rec_result:
        rec_result[rec_result.index('pair')] = 'pear'
    if 'pairs' in rec_result:
        rec_result[rec_result.index('pairs')] = 'pear'
    if 'pitch' in rec_result:
        rec_result[rec_result.index('pitch')] = 'peach'
    if 'picture' in rec_result:
        rec_result[rec_result.index('picture')] = 'peach'
    if 'piece' in rec_result:
        rec_result[rec_result.index('piece')] = 'peach'
#correction for navi
    if 'photo' in rec_result:
        rec_result[rec_result.index('photo')] = 'follow'
    if ('raising' in rec_result or 'raised' in rec_result) or ('writing' in rec_result or 'raises' in rec_result) or ('waiting' in rec_result):
        if 'left' in rec_result:
            rec_result[rec_result.index('left')]='raising left'
        if 'right' in rec_result:
            rec_result[rec_result.index('right')]='raising right'
    if 'pointing' in rec_result or 'pointed' in rec_result:
        if 'left' in rec_result:
            rec_result[rec_result.index('left')]='pointing left'
        if 'right' in rec_result:
            rec_result[rec_result.index('right')]='pointing right'
    if 'agenda' in rec_result:
        rec_result[rec_result.index('agenda')]='gender'
    if 'seeking' in rec_result:
        rec_result[rec_result.index('seeking')] ='sitting'
    if 'seating' in rec_result:
        rec_result[rec_result.index('seating')] ='sitting'
    if 'men' in rec_result:
        rec_result[rec_result.index('men')] = 'man'
    if 'wave' in rec_result:
        rec_result[rec_result.index('wave')] = 'raising right'
    if 'waving' in rec_result:
        rec_result[rec_result.index('waving')] = 'raising right'
    if 'bathroom' in rec_result:
        rec_result[rec_result.index('bathroom')] = 'bedroom'
    if 'jester' in rec_result:
        rec_result[rec_result.index('jester')] = 'gesture'
    if 'chester' in rec_result:
        rec_result[rec_result.index('chester')] = 'gesture'
    if 'locate' in rec_result:
        rec_result[rec_result.index('locate')] = 'find'
    if 'ice' in rec_result:
        rec_result[rec_result.index('ice')] = 'ice tea'
    if 'tea' in rec_result:
        rec_result[rec_result.index('tea')] = 'ice tea'
    if 'orange' in rec_result:
        rec_result[rec_result.index('orange')] = 'orange juice'
    if 'juice' in rec_result:
        rec_result[rec_result.index('juice')] = 'orange juice'
    if 'sofa' in rec_result or 'silver' in rec_result:
        if 'left' in rec_result:
            rec_result[rec_result.index('left')]='leftsofa'
        if 'right' in rec_result:
            rec_result[rec_result.index('right')]='rightsofa'
    if 'dressing' in rec_result or 'dress' in rec_result:
        rec_result[rec_result.index('sofa')]='dressingtable'
    if 'white' in rec_result or 'wait' in rec_result:
        rec_result[rec_result.index('white')]='whitetable'
    if 'wait' in rec_result:
        rec_result[rec_result.index('wait')]='whitetable'
    

