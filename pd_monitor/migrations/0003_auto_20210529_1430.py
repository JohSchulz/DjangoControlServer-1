# Generated by Django 3.2.3 on 2021-05-29 12:30

from django.conf import settings
from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):

    dependencies = [
        migrations.swappable_dependency(settings.AUTH_USER_MODEL),
        ('pd_monitor', '0002_auto_20210529_1404'),
    ]

    operations = [
        migrations.AddField(
            model_name='device',
            name='added_by',
            field=models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.CASCADE, related_name='devices', to=settings.AUTH_USER_MODEL),
        ),
        migrations.AddField(
            model_name='device',
            name='description',
            field=models.CharField(blank=True, max_length=100),
        ),
        migrations.AddField(
            model_name='device',
            name='ip_address',
            field=models.CharField(blank=True, max_length=20),
        ),
        migrations.AddField(
            model_name='device',
            name='port',
            field=models.CharField(blank=True, max_length=4),
        ),
        migrations.AlterField(
            model_name='device',
            name='value',
            field=models.FloatField(null=True),
        ),
    ]
